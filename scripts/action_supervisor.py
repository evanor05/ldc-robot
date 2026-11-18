"""Track remote Action termination separately from local motion permission.

No ROS imports: clients/handles/futures are supplied by the ROS owner. A cancel
reply only acknowledges a request; only rejection or a terminal result settles
an owned goal. Revoked late acceptances are canceled instead of being dropped.
"""
from collections import deque
from concurrent.futures import Future
from dataclasses import dataclass


@dataclass
class _Goal:
    token: str
    ros_id: str
    handle: object = None
    revoked: bool = False
    cancel_requested: bool = False
    cancel_code: int | None = None
    error: str = ''
    outcome: str | None = None


class ActionSupervisor:
    def __init__(self, record, before_send):
        self._pending = {}
        self._history = deque(maxlen=32)
        self._record = record
        self._before_send = before_send

    @property
    def quiet(self):
        return not self._pending

    def require_quiet(self):
        if not self.quiet:
            raise ValueError('旧目标终态尚未确认，运动保持锁定，请等待取消确认')

    def snapshot(self):
        return {'cancelConfirmed': self.quiet,
                'pending': [self._describe(g) for g in self._pending.values()],
                'history': list(self._history)}

    @staticmethod
    def _describe(goal):
        return {'token': goal.token, 'rosGoalId': goal.ros_id,
                'accepted': goal.handle is not None, 'revoked': goal.revoked,
                'cancelRequested': goal.cancel_requested,
                'cancelReturnCode': goal.cancel_code, 'remoteOutcome': goal.outcome, 'error': goal.error}

    def _settle(self, goal, outcome):
        goal.outcome = outcome
        # The owner persists terminal evidence before the in-memory barrier is
        # removed. A failed write must not silently erase the recovery record.
        self._record('action_terminal', outcome, goalId=goal.token, rosGoalId=goal.ros_id)
        self._pending.pop(goal.token, None)
        self._history.append({**self._describe(goal), 'outcome': outcome})

    def send(self, client, goal, token, feedback, accepted, completed, *, goal_uuid):
        self.require_quiet()
        try:
            identifier = bytes(goal_uuid.uuid)
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError('目标 UUID 必须包含 16 字节') from error
        if len(identifier) != 16 or not any(identifier):
            raise ValueError('目标 UUID 必须是非零的 16 字节标识')
        owned = _Goal(token, ros_id=identifier.hex())
        self._pending[token] = owned
        try:
            self._before_send(owned.ros_id, token)
        except Exception:
            self._pending.pop(token, None)  # No RPC was attempted.
            raise
        try:
            pending = client.send_goal_async(goal, feedback_callback=feedback, goal_uuid=goal_uuid)
        except Exception as error:
            # Without a server response, do not infer that nothing was sent.
            owned.error = f'发送结果未确认：{error}'
            owned.revoked = True
            raise
        self._record('action_sent', '发送请求已提交，等待远端受理',
                     goalId=token, rosGoalId=owned.ros_id)

        def on_accepted(done):
            try:
                handle = done.result()
            except Exception as error:
                owned.error = f'受理结果未确认：{error}'
                owned.revoked = True
                accepted(done, token)
                return
            if not handle.accepted:
                try:
                    self._settle(owned, 'rejected')
                except Exception as error:
                    owned.error = f'终态证据未持久保存：{error}'
                    failed = Future()
                    failed.set_exception(error)
                    accepted(failed, token)
                else:
                    accepted(done, token)
                return
            owned.handle = handle
            # Keep the UUID recorded before sending, including when acceptance
            # is delayed/lost. rclpy builds its handle from this request UUID.
            accepted(done, token)
            try:
                result = handle.get_result_async()
                result.add_done_callback(on_result)
            except Exception as error:
                owned.error = f'终态订阅失败：{error}'
                owned.revoked = True
                failed = Future()
                failed.set_exception(error)
                completed(failed, token)
            if owned.revoked:
                self._cancel(owned)

        def on_result(done):
            try:
                status = done.result().status
                if status not in (4, 5, 6):
                    raise ValueError(f'非终态结果：{status}')
            except Exception as error:
                owned.error = f'终态未确认：{error}'
                owned.revoked = True
                self._cancel(owned)
            else:
                try:
                    self._settle(owned, {4: 'succeeded', 5: 'canceled', 6: 'aborted'}[status])
                except Exception as error:
                    owned.error = f'终态证据未持久保存：{error}'
                    failed = Future()
                    failed.set_exception(error)
                    completed(failed, token)
                    return
            completed(done, token)

        pending.add_done_callback(on_accepted)

    def revoke_all(self):
        for goal in list(self._pending.values()):
            goal.revoked = True
            if goal.outcome is not None:
                try:
                    self._settle(goal, goal.outcome)
                except Exception as error:
                    goal.error = f'终态证据未持久保存：{error}'
            else:
                self._cancel(goal)

    def _cancel(self, goal):
        if goal.handle is None or goal.cancel_requested or goal.token not in self._pending:
            return
        goal.cancel_requested = True
        self._record('action_cancel_requested', '已请求取消；等待远端终态',
                     goalId=goal.token, rosGoalId=goal.ros_id)
        try:
            future = goal.handle.cancel_goal_async()
            if future is not None:
                future.add_done_callback(lambda done: self._cancel_reply(goal, done))
        except Exception as error:
            goal.error = f'取消请求失败：{error}'
            goal.cancel_requested = False

    def _cancel_reply(self, goal, future):
        try:
            goal.cancel_code = int(future.result().return_code)
            if goal.cancel_code != 0:
                goal.error = f'取消未获受理：return_code={goal.cancel_code}'
                goal.cancel_requested = False
        except Exception as error:
            goal.error = f'取消响应未确认：{error}'
            goal.cancel_requested = False
        self._record('action_cancel_reply', goal.error or '取消已受理，仍等待远端终态',
                     goalId=goal.token, rosGoalId=goal.ros_id,
                     returnCode=goal.cancel_code)
