"""Startup recovery of unconfirmed action goals; no ROS or motion side effects.

The ROS owner supplies a transport (GetResult / UUID-targeted CancelGoal keyed
by the *recorded* old action name) and drives tick() on its executor thread.
Discipline (decision.md 2026-11-17, COORDINATION.md S1 Q2/Q3, adjudication
20261118-b-contract.md):

- Only a terminal result (status 4/5/6) or a rejection persisted before the
  crash settles a goal, and only after the terminal write succeeds.
- Cancel ACKs, code0+empty lists, UNKNOWN(0), non-terminal statuses, timeouts,
  service absence, silence and process exit never settle anything.
- Recovery never sends goals and never issues CancelAll (zero UUID).
- isolated_unknown requires explicit human abandonment plus verified isolation
  evidence from the runtime owner; cancelConfirmed stays false while any
  isolated/unconfirmed row exists. motionIsolated=true is recorded separately.
- Isolation evidence is bound to a frozen coverage set (runs/goals) plus a
  verifiedAt watermark; current-boot goals and goals prepared after the
  watermark are NEVER covered by old evidence (adjudication item ①/⑥).
- The full evidence package is persisted immutably (sha256-named file) with a
  reference in the journal; when unwritable, a persistent audit-gap marker is
  written and the snapshot reports evidencePersisted=false (item ②).
- An isolated goal whose terminal write fails keeps motionIsolated and does
  NOT re-raise the motion barrier (item ③, T15: current-run gate only checks
  current pending).
- Page refresh / backend restart / sensor recovery are not events here: the
  gate only moves on explicit calls, and nothing re-dispatches an old goal.
- tick() contains unexpected exceptions fail-closed: the barrier never drops
  because of an internal error (item ⑤).

Naming map (item ④): journal metadata uses bootId/runEpoch (bridge schema);
B's begin/registry uses bootId/epoch. set_current_identity(epoch=...) is the
explicit adapter; _Record.run_epoch compares against identity['epoch'].
"""
import hashlib
import json
import os
from pathlib import Path
import time

TERMINAL_BY_STATUS = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}
ACTIVE_STATUSES = (1, 2, 3)
RETRY_INTERVAL = 2.0
RETRY_MAX_INTERVAL = 30.0
REQUEST_TIMEOUT = 10.0
EVIDENCE_MAX_AGE = 300.0  # seconds; stale evidence is rejected (adjudication R2)

# Per-record recovery classifications (audit doc §1.1).
RECOVERY_STATES = (
    'unknown_pending', 'querying', 'cancel_requested', 'cancel_acknowledged',
    'terminal_confirmed', 'terminal_persist_failed', 'cannot_confirm',
    'isolated_unknown',
)
_SETTLED = ('terminal_confirmed', 'isolated_unknown')


class _Record:
    def __init__(self, row):
        metadata = row['metadata']
        self.uuid = row['goalUUID']
        self.metadata = metadata
        self.prepared_at = row['preparedAt']
        self.action_name = metadata.get('actionName') or ''
        self.boot_id = metadata.get('bootId')
        self.run_epoch = metadata.get('runEpoch')  # journal schema key; B's 'epoch' maps here
        self.state = 'unknown_pending'
        self.sub_reason = ''
        self.known_outcome = None
        self.cancel_requested = False
        self.cancel_code = None
        self.name_collision_risk = False
        self.motion_isolated = False
        self.isolation_evidence = None
        self.result_future = None
        self.cancel_future = None
        self.late_futures = []
        self.result_issued_at = 0.0
        self.cancel_issued_at = 0.0
        self.next_attempt_at = 0.0
        self.attempts = 0
        self.last_error = ''

    def describe(self):
        return {'goalUUID': self.uuid, 'actionName': self.action_name,
                'bootId': self.boot_id, 'runEpoch': self.run_epoch,
                'preparedAt': self.prepared_at, 'recoveryState': self.state,
                'subReason': self.sub_reason, 'cancelRequested': self.cancel_requested,
                'cancelReturnCode': self.cancel_code, 'knownOutcome': self.known_outcome,
                'nameCollisionRisk': self.name_collision_risk,
                'motionIsolated': self.motion_isolated,
                'target': self.metadata.get('target'), 'mapPath': self.metadata.get('mapPath'),
                'lastError': self.last_error}


def _evidence_error(evidence):
    """Return a rejection reason for an isolation evidence package, or ''.

    Fail-closed per adjudication-20261118-b-contract.md R2: bound to the lock
    holding bridge, old boot+runEpoch, domain, graph collection time/endpoints,
    full old-process set with identity verification; allOldRuns must carry a
    verifiable coverage set, never a bare wildcard boolean.
    """
    if not isinstance(evidence, dict):
        return '隔离证据必须为对象'
    if evidence.get('graphVerified') is not True:
        return '缺少运行图核验（graphVerified=true）'
    if not evidence.get('graphCollectedAt'):
        return '缺少运行图采集时间（graphCollectedAt）'
    if not isinstance(evidence.get('graphEndpoints'), list):
        return '缺少运行图端点/GID 清单（graphEndpoints，可为空列表=旧端点已消失）'
    if evidence.get('oldVelocityPathToFinalExit') is not False:
        return '必须证明旧速度源到最终出口无路径（oldVelocityPathToFinalExit=false）'
    if not evidence.get('uniqueFinalPublisher'):
        return '缺少唯一最终发布者身份（uniqueFinalPublisher）'
    if not evidence.get('bridgeBootId'):
        return '缺少当前持锁 Bridge 身份（bridgeBootId）'
    if not evidence.get('bridgeLockPath'):
        return '缺少当前持锁 Bridge 锁路径绑定（bridgeLockPath）'
    if evidence.get('domain') is None:
        return '缺少 ROS 域绑定（domain）'
    if not evidence.get('abandonSessionId'):
        return '缺少显式放弃请求引用（abandonSessionId）'
    if not isinstance(evidence.get('velocityChain'), list) or not evidence.get('velocityChain'):
        return '缺少完整旧速度链路清单（velocityChain，非空列表）'
    processes = evidence.get('oldProcesses')
    if not isinstance(processes, list) or not processes:
        return '缺少旧进程组处置清单（oldProcesses，不得编造占位进程）'
    for item in processes:
        if not isinstance(item, dict) or item.get('disposition') not in ('terminated', 'quarantined', 'process_gone'):
            return '旧进程处置必须为 terminated/quarantined/process_gone'
        if not item.get('pid'):
            return '旧进程处置必须附 pid'
        # 字段名对齐产品代码 identity().start（/proc/stat field22）；
        # startTicks 为过渡期兼容别名（B 文档前版命名，§B.3 已更正）。
        start = item.get('start') or item.get('startTicks')
        if item.get('disposition') in ('terminated', 'quarantined') and not start:
            return 'terminated/quarantined 处置必须附 start（/proc/stat field22 身份比对）'
        if item.get('disposition') == 'terminated':
            if item.get('exitStatus') is None:
                return 'terminated 处置必须附真实 wait 退出状态'
            # managedProcess.js terminateProcess 只读 proc.pid、父死后后代不可追踪
            # → terminated 必须附身份校验与后代处置证据，fail-closed。
            if item.get('identityVerified') is not True:
                return 'terminated 处置必须附身份校验（identityVerified=true，registry pid+startTicks 比对）'
            if not isinstance(item.get('descendants'), list) and not item.get('descendantsEvidenceRef'):
                return 'terminated 处置必须附后代处置清单（descendants，可为空列表）或证据引用（descendantsEvidenceRef）'
            for child in item.get('descendants') or []:
                if not isinstance(child, dict) or not child.get('pid') or \
                        child.get('disposition') not in ('terminated', 'quarantined', 'process_gone'):
                    return '后代处置必须为 terminated/quarantined/process_gone 且附 pid'
                if child.get('disposition') == 'terminated' and child.get('exitStatus') is None:
                    return 'terminated 后代必须附退出状态'
    if not evidence.get('verifiedAt'):
        return '缺少核验时间（verifiedAt）'
    if evidence.get('allOldRuns') is True:
        covered = evidence.get('coveredRuns')
        if not isinstance(covered, list) or not covered:
            return 'allOldRuns 必须携带可核对的完整覆盖集（coveredRuns=[{bootId,runEpoch},...]）'
        for run in covered:
            if not isinstance(run, dict) or not run.get('bootId') or not run.get('runEpoch'):
                return 'coveredRuns 每项必须含 bootId+runEpoch'
    elif not (evidence.get('bootId') and evidence.get('runEpoch')):
        return '证据必须指明 bootId+runEpoch 或 allOldRuns=true+coveredRuns'
    return ''


class RecoveryGate:
    """Run-level motion barrier + per-goal recovery state machine.

    journal: an ActionJournal (or compatible fake). transport: object with
    get_result(action_name, uuid_hex) and cancel_goal(action_name, uuid_hex)
    returning futures whose result() exposes .status / .return_code and
    .goals_canceling. record(kind, reason, **extra): owner event log.
    clock: monotonic drive clock; wall_clock: evidence freshness reference.
    """

    def __init__(self, journal, *, record=None, clock=time.monotonic, wall_clock=time.time):
        self._journal = journal
        self._record_cb = record if record is not None else (lambda *a, **k: None)
        self._clock = clock
        self._wall_clock = wall_clock
        self._transport = None
        self._records = {}
        self._loaded = False
        self._journal_error = ''
        self._audit_gap = False
        self._abandon = None
        self._isolated_release = False  # journal-fault path released by evidence
        self._release_coverage = None   # frozen coverage set + watermark (items ①/⑥)
        self._release_evidence = None
        self._evidence_persisted = True  # False when the full package could not be stored
        self._current_identity = None
        self._state = 'unloaded'
        self._barrier = True  # fail closed until a healthy load proves otherwise
        self._cancel_confirmed = False

    # ---- owner wiring ----

    def set_current_identity(self, boot_id, epoch, action_name, domain=None):
        """B begin identity adapter: begin 'epoch' == journal metadata 'runEpoch'."""
        self._current_identity = {'bootId': boot_id, 'epoch': epoch,
                                  'actionName': action_name, 'domain': domain}
        for rec in self._records.values():
            self._annotate_collision(rec)

    def attach_transport(self, transport):
        self._transport = transport

    def _annotate_collision(self, rec):
        identity = self._current_identity
        rec.name_collision_risk = bool(
            identity and rec.action_name and rec.action_name == identity['actionName'] and
            (rec.boot_id != identity['bootId'] or rec.run_epoch != identity['epoch']))

    # ---- coverage (adjudication items ①/⑥) ----

    def _covers(self, rec):
        """Frozen coverage set + verifiedAt watermark; current boot never covered."""
        coverage = self._release_coverage
        if coverage is None:
            return False
        identity = self._current_identity
        if identity and rec.boot_id == identity['bootId']:
            return False  # current-run pending is never released by old evidence
        if rec.prepared_at > coverage['verifiedAt']:
            return False  # prepared after the evidence watermark
        if rec.uuid in coverage['goals']:
            return True
        return (rec.boot_id, rec.run_epoch) in coverage['runs']

    # ---- load / barrier ----

    def load(self):
        """Idempotent startup load. Establishes the barrier before any enable."""
        try:
            rows = self._journal.pending()
        except Exception as error:
            self._journal_error = str(error)
            self._loaded = False
            self._audit_gap = True
            self._record_cb('recovery_journal_fault', f'账本不可加载：{error}')
            self._recompute()
            return self.snapshot()
        self._journal_error = ''
        self._loaded = True
        seen = set()
        for row in rows:
            uuid = row['goalUUID']
            seen.add(uuid)
            if uuid not in self._records:
                rec = _Record(row)
                self._annotate_collision(rec)
                if self._covers(rec):
                    # Reappearing row inside the frozen coverage of a prior
                    # verified isolation (e.g. journal repaired after release).
                    rec.state = 'isolated_unknown'
                    rec.sub_reason = 'isolated_unknown'
                    rec.motion_isolated = True
                    rec.isolation_evidence = self._release_evidence
                elif not rec.action_name:
                    rec.state = 'cannot_confirm'
                    rec.sub_reason = 'missing_action_name'
                    self._audit_gap = True
                self._records[uuid] = rec
        for uuid in list(self._records):
            if uuid not in seen:
                # Settled elsewhere (e.g. live supervisor) between loads.
                self._records.pop(uuid)
        self._record_cb('recovery_loaded', f'加载未确认目标 {len(self._records)} 条',
                        pending=len(self._records))
        self._recompute()
        return self.snapshot()

    # ---- recovery drive ----

    def tick(self):
        """Drive queries/cancels; called by the ROS owner on its executor.

        Fail-closed: an unexpected internal error keeps the barrier and is
        recorded; it never releases motion (adjudication item ⑤).
        isolated_unknown records keep cancel tracking (decision.md): a late
        terminal still settles them, but they never re-raise the barrier.
        """
        now = self._clock()
        for rec in list(self._records.values()):
            if rec.state in ('terminal_confirmed', 'terminal_persist_failed'):
                continue
            try:
                self._drive_record(rec, now)
            except Exception as error:
                rec.last_error = f'恢复驱动异常：{error}'
                self._record_cb('recovery_tick_error', str(error), goalUUID=rec.uuid)
        self._recompute()
        return self.snapshot()

    def _drive_record(self, rec, now):
        self._drain_futures(rec, now)
        if rec.state in ('terminal_confirmed', 'terminal_persist_failed'):
            return
        if not rec.action_name or self._transport is None:
            return
        if rec.state == 'cancel_acknowledged':
            if rec.result_future is None:
                self._issue_result(rec, now)
            return
        if rec.state in ('unknown_pending', 'cannot_confirm', 'querying',
                         'cancel_requested', 'isolated_unknown'):
            if rec.result_future is None and now >= rec.next_attempt_at:
                self._issue_result(rec, now)

    def _backoff(self, rec, now):
        rec.attempts += 1
        rec.next_attempt_at = now + min(RETRY_MAX_INTERVAL,
                                        RETRY_INTERVAL * (2 ** min(rec.attempts, 4)))

    def _issue_result(self, rec, now):
        try:
            future = self._transport.get_result(rec.action_name, rec.uuid)
        except Exception as error:
            self._set_state(rec, 'cannot_confirm', 'service_missing')
            rec.last_error = str(error)
            self._backoff(rec, now)
            return
        rec.result_future = future
        rec.result_issued_at = now
        if rec.state in ('unknown_pending', 'cannot_confirm'):
            rec.state = 'querying'
        self._evidence(rec, 'recovery_query', {'actionName': rec.action_name})

    def _issue_cancel(self, rec, now):
        if rec.cancel_requested:
            return
        try:
            future = self._transport.cancel_goal(rec.action_name, rec.uuid)
        except Exception as error:
            self._set_state(rec, 'cannot_confirm', 'cancel_service_missing')
            rec.last_error = str(error)
            self._backoff(rec, now)
            return
        rec.cancel_future = future
        rec.cancel_issued_at = now
        rec.cancel_requested = True
        self._set_state(rec, 'cancel_requested')
        self._evidence(rec, 'recovery_cancel', {'actionName': rec.action_name})

    def _set_state(self, rec, state, sub_reason=None):
        # isolated_unknown is sticky: only a persisted terminal may replace it.
        if rec.state == 'isolated_unknown' and state not in ('terminal_confirmed', 'terminal_persist_failed'):
            return
        rec.state = state
        if sub_reason is not None:
            rec.sub_reason = sub_reason

    def _drain_futures(self, rec, now):
        # Late responses after a declared timeout are still valid evidence.
        for kind, future in list(rec.late_futures):
            if future.done():
                rec.late_futures.remove((kind, future))
                if rec.state in ('terminal_confirmed', 'terminal_persist_failed'):
                    continue
                if kind == 'result':
                    self._on_result(rec, future, now)
                else:
                    self._on_cancel(rec, future, now)
        if len(rec.late_futures) > 4:
            rec.late_futures = rec.late_futures[-4:]
        future = rec.result_future
        if future is not None:
            if future.done():
                rec.result_future = None
                self._on_result(rec, future, now)
            elif now - rec.result_issued_at > REQUEST_TIMEOUT:
                rec.late_futures.append(('result', future))
                rec.result_future = None
                self._set_state(rec, 'cannot_confirm', 'result_timeout')
                self._backoff(rec, now)
        future = rec.cancel_future
        if future is not None:
            if future.done():
                rec.cancel_future = None
                self._on_cancel(rec, future, now)
            elif now - rec.cancel_issued_at > REQUEST_TIMEOUT:
                rec.late_futures.append(('cancel', future))
                rec.cancel_future = None
                rec.cancel_requested = False
                self._set_state(rec, 'cannot_confirm', 'cancel_timeout')
                self._backoff(rec, now)

    def _on_result(self, rec, future, now):
        try:
            status = int(future.result().status)
        except Exception as error:
            self._set_state(rec, 'cannot_confirm', 'result_error')
            rec.last_error = str(error)
            self._backoff(rec, now)
            return
        self._evidence(rec, 'recovery_result', {'status': status,
                                                'nameCollisionRisk': rec.name_collision_risk})
        if status in TERMINAL_BY_STATUS:
            self._settle(rec, TERMINAL_BY_STATUS[status])
            return
        if status == 0:
            # Unknown to this server: never accepted / rejected / restarted /
            # result cache expired / a different generation owns the name.
            self._set_state(rec, 'cannot_confirm', 'result_unknown_or_expired')
            if not rec.cancel_requested:
                self._issue_cancel(rec, now)
            self._backoff(rec, now)
            return
        if status in ACTIVE_STATUSES:
            if rec.state != 'isolated_unknown':
                rec.sub_reason = f'remote_status_{status}'
            if status in (1, 2) and not rec.cancel_requested:
                self._issue_cancel(rec, now)
            if rec.state not in ('cancel_acknowledged', 'isolated_unknown'):
                rec.state = 'querying' if status != 3 else 'cancel_requested'
            self._backoff(rec, now)
            return
        self._set_state(rec, 'cannot_confirm', f'unexpected_status_{status}')
        self._backoff(rec, now)

    def _on_cancel(self, rec, future, now):
        try:
            reply = future.result()
            code = int(reply.return_code)
            canceling = [bytes(info.goal_id.uuid).hex() for info in reply.goals_canceling]
        except Exception as error:
            rec.cancel_requested = False
            self._set_state(rec, 'cannot_confirm', 'cancel_error')
            rec.last_error = str(error)
            self._backoff(rec, now)
            return
        rec.cancel_code = code
        self._evidence(rec, 'recovery_cancel_reply', {'returnCode': code, 'goalsCanceling': canceling})
        if code == 0 and rec.uuid in canceling:
            # ACK only: the goal moved to CANCELING; terminal comes from result.
            self._set_state(rec, 'cancel_acknowledged', '')
            if rec.result_future is None:
                self._issue_result(rec, now)
            return
        reason = {0: 'cancel_ack_empty_list', 1: 'cancel_rejected',
                  2: 'cancel_unknown_goal', 3: 'cancel_goal_terminated'}.get(code, f'cancel_code_{code}')
        self._set_state(rec, 'cannot_confirm', reason)
        if code == 3 and rec.result_future is None:
            # Already terminal server-side, but which one? Try the result cache.
            self._issue_result(rec, now)
        self._backoff(rec, now)

    def _settle(self, rec, outcome):
        try:
            self._journal.terminal(rec.uuid, outcome)
        except Exception as error:
            rec.state = 'terminal_persist_failed'
            rec.known_outcome = outcome
            rec.last_error = str(error)
            # An isolated goal stays motion-isolated: the barrier must not rise
            # again because of a storage failure (adjudication item ③).
            self._record_cb('recovery_persist_failed', f'终态落盘失败：{error}',
                            goalUUID=rec.uuid, outcome=outcome,
                            motionIsolated=rec.motion_isolated)
            return
        rec.state = 'terminal_confirmed'
        rec.known_outcome = outcome
        rec.sub_reason = ''
        self._record_cb('recovery_settled', f'旧目标终态已确认并落盘：{outcome}',
                        goalUUID=rec.uuid, outcome=outcome)

    def retry_persist(self, uuid):
        """Explicit retry of a failed terminal write; never auto-resumes."""
        rec = self._records.get(uuid)
        if rec is None or rec.state != 'terminal_persist_failed' or not rec.known_outcome:
            raise ValueError('该目标没有待重试的终态落盘')
        self._settle(rec, rec.known_outcome)
        self._recompute()
        return self.snapshot()

    # ---- explicit abandonment + isolation ----

    def abandon_old_run(self, session_id, request_id=''):
        if not session_id:
            raise ValueError('放弃旧轮必须由控制页 owner 会话显式发起')
        if self._state not in ('recovery_barrier', 'journal_fault', 'abandon_requested'):
            raise ValueError('当前没有需要放弃的未确认旧轮')
        if self._abandon is None:
            self._abandon = {'sessionId': session_id, 'requestId': request_id, 'at': time.time()}
            self._record_cb('recovery_abandon_requested',
                            '显式放弃旧轮：撤权→零速→等待旧运动源隔离证据',
                            sessionId=session_id, requestId=request_id)
        self._recompute()
        return self.snapshot()

    def cancel_abandon(self, session_id):
        if self._abandon is None:
            raise ValueError('没有进行中的放弃请求')
        if any(rec.motion_isolated for rec in self._records.values()) or self._isolated_release:
            raise ValueError('隔离证据已生效，不能撤回放弃')
        self._abandon = None
        self._record_cb('recovery_abandon_cancelled', '放弃请求已撤回，继续恢复查询', sessionId=session_id)
        self._recompute()
        return self.snapshot()

    def _check_evidence_binding(self, evidence):
        """Runtime binding checks beyond the static schema (adjudication R2)."""
        identity = self._current_identity
        now = self._wall_clock()
        verified = evidence.get('verifiedAt')
        if not isinstance(verified, (int, float)) or not (now - EVIDENCE_MAX_AGE <= verified <= now + 5):
            return '隔离证据过期或时间异常（verifiedAt 必须为 300s 水位内的 epoch 秒数值）'
        abandon = self._abandon
        if evidence.get('abandonSessionId') != abandon['sessionId']:
            return '证据未引用当前显式放弃请求（abandonSessionId 不一致）'
        if abandon.get('requestId') and evidence.get('abandonRequestId') != abandon['requestId']:
            return '证据 abandonRequestId 与当前放弃请求不一致'
        if identity:
            if evidence.get('bridgeBootId') != identity['bootId']:
                return '证据未绑定当前持锁 Bridge（bridgeBootId 不一致）'
            if identity.get('domain') is not None and evidence.get('domain') != identity['domain']:
                return '证据域绑定不一致（domain）'
            if evidence.get('bootId') == identity['bootId'] and evidence.get('runEpoch') == identity['epoch']:
                return '不得隔离当前轮（身份冲突）'
            for run in evidence.get('coveredRuns') or []:
                if run.get('bootId') == identity['bootId'] and run.get('runEpoch') == identity['epoch']:
                    return 'coveredRuns 不得包含当前轮（身份冲突）'
        return ''

    def _persist_evidence_package(self, evidence):
        """Store the full package immutably; return (ref, sha256, persisted).

        File journals: sha256-named write-once file beside the journal, plus a
        persistent audit-gap marker on any failure. :memory: journals (tests)
        keep the package in memory only and say so.
        """
        canonical = json.dumps(evidence, sort_keys=True, ensure_ascii=False, allow_nan=False)
        digest = hashlib.sha256(canonical.encode()).hexdigest()
        filename = getattr(self._journal, 'filename', ':memory:')
        if filename == ':memory:':
            return 'memory://' + digest, digest, 'memory'
        directory = Path(filename).parent
        target = directory / f'isolation-evidence-{digest[:16]}.json'
        try:
            directory.mkdir(parents=True, exist_ok=True, mode=0o700)
            try:
                with open(target, 'x', encoding='utf-8') as handle:
                    handle.write(canonical)
                    handle.flush()
                    os.fsync(handle.fileno())
            except FileExistsError:
                if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
                    raise ValueError('同名隔离证据内容冲突，拒绝覆盖')
            return str(target), digest, True
        except ValueError:
            raise  # identity conflict is a hard rejection, not a storage gap
        except Exception as error:
            self._audit_gap = True
            self._append_audit_gap({'kind': 'isolation_evidence_unpersisted',
                                    'evidenceSha256': digest, 'error': str(error)})
            self._record_cb('recovery_evidence_unpersisted',
                            f'隔离证据完整包无法持久化：{error}', evidenceSha256=digest)
            return 'unpersisted://' + digest, digest, False

    def _append_audit_gap(self, entry):
        """Persistent audit-gap marker (not just an in-memory event)."""
        filename = getattr(self._journal, 'filename', ':memory:')
        if filename == ':memory:':
            return
        try:
            path = Path(filename).parent / 'recovery-audit-gap.jsonl'
            with open(path, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps({'at': time.time(), **entry}, ensure_ascii=False) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        except Exception as error:
            self._record_cb('recovery_audit_gap_unpersisted', str(error))

    def submit_isolation_evidence(self, evidence):
        """B's proof that the old motion source is isolated (audit doc R2)."""
        if self._abandon is None:
            raise ValueError('必须先显式放弃旧轮，才能提交隔离证据')
        reason = _evidence_error(evidence) or self._check_evidence_binding(evidence)
        if reason:
            self._record_cb('recovery_isolation_rejected', reason)
            raise ValueError('隔离证据不足：' + reason)
        # Freeze the coverage set + watermark BEFORE applying (items ①/⑥).
        if evidence.get('allOldRuns') is True:
            runs = {(run['bootId'], run['runEpoch']) for run in evidence['coveredRuns']}
        else:
            runs = {(evidence['bootId'], evidence['runEpoch'])}
        coverage = {'runs': runs,
                    'goals': set(evidence.get('coveredGoals') or []),
                    'verifiedAt': evidence['verifiedAt']}
        ref, digest, persisted = self._persist_evidence_package(evidence)
        self._release_coverage = coverage
        self._release_evidence = evidence
        self._evidence_persisted = persisted
        covered = []
        for rec in self._records.values():
            if rec.state in _SETTLED or not self._covers(rec):
                continue
            rec.motion_isolated = True
            rec.isolation_evidence = evidence
            rec.state = 'isolated_unknown'  # cancelConfirmed stays false while isolated
            rec.sub_reason = 'isolated_unknown'
            covered.append(rec.uuid)
            self._evidence(rec, 'recovery_isolated', {
                'motionIsolated': True, 'cancelConfirmed': False,
                'evidenceRef': ref, 'evidenceSha256': digest,
                'verifiedAt': evidence.get('verifiedAt'),
                'namespace': evidence.get('namespace'),
                'oldServerDestroyedAt': evidence.get('oldServerDestroyedAt')})
        if not self._records:
            # Journal fault: nothing readable; allOldRuns with a verifiable
            # coverage set releases the barrier but the audit gap stays.
            if evidence.get('allOldRuns') is not True:
                raise ValueError('账本不可读时需要 allOldRuns=true+coveredRuns 的隔离证据')
            self._isolated_release = True
            self._audit_gap = True
        self._record_cb('recovery_isolated',
                        f'隔离证据核验通过，{len(covered)} 条旧目标记为 isolated_unknown',
                        covered=covered, verifiedAt=evidence.get('verifiedAt'),
                        evidenceRef=ref, evidenceSha256=digest, evidencePersisted=persisted)
        self._recompute()
        return self.snapshot()

    # ---- gate outputs ----

    def _recompute(self):
        # motion_isolated records never re-raise the barrier, even when their
        # terminal write failed (item ③); current-run gate = supervisor domain.
        active = [rec for rec in self._records.values()
                  if rec.state not in _SETTLED and not rec.motion_isolated]
        isolated = [rec for rec in self._records.values() if rec.motion_isolated]
        if not self._loaded and not self._isolated_release:
            self._state = 'journal_fault' if self._journal_error else 'unloaded'
            self._barrier = True
            self._cancel_confirmed = False
            return
        if active and self._abandon is not None:
            self._state = 'abandon_requested'
        elif active:
            self._state = 'recovery_barrier'
        elif isolated or self._isolated_release:
            self._state = 'old_run_isolated'
        elif self._records:
            self._state = 'recovered_idle'
        else:
            self._state = 'clean_idle'
        self._barrier = bool(active)
        # cancelConfirmed = old ledger fully terminal-confirmed: no pending,
        # no isolated_unknown, no failed persist. Current-run confirmation is
        # the supervisor's quiet flag; the bridge combines both.
        self._cancel_confirmed = (self._loaded and not self._isolated_release
                                  and not any(rec.state != 'terminal_confirmed'
                                              for rec in self._records.values()))

    @property
    def barrier(self):
        return self._barrier

    @property
    def motion_allowed(self):
        return not self._barrier

    @property
    def prepare_allowed(self):
        return (not self._barrier and self._loaded and not self._journal_error
                and self._state in ('clean_idle', 'recovered_idle', 'old_run_isolated'))

    def _allowed_ops(self):
        if self._state in ('journal_fault', 'unloaded'):
            return ['view_audit_gap', 'retry_journal_load', 'abandon_old_run', 'estop', 'stop']
        if self._state == 'abandon_requested':
            return ['wait_isolation', 'submit_isolation_evidence', 'cancel_abandon', 'estop', 'stop']
        if self._barrier:
            ops = ['view_pending', 'retry_query', 'abandon_old_run', 'estop', 'stop']
            if any(rec.state == 'terminal_persist_failed' for rec in self._records.values()):
                ops.insert(2, 'retry_persist')
            return ops
        if not self._loaded or self._journal_error:
            return ['repair_journal', 'enable', 'estop', 'stop']
        return ['prepare_new_run', 'enable', 'estop', 'stop']

    def _reason(self):
        if self._state in ('journal_fault', 'unloaded'):
            return f'目标账本不可用（{self._journal_error or "尚未加载"}），可能存在未确认旧目标，运动保持锁定'
        if self._state == 'abandon_requested':
            return '已显式放弃旧轮，等待旧运动源隔离证据；证据核验通过前运动保持锁定'
        if self._barrier:
            return f'存在 {sum(1 for r in self._records.values() if r.state not in _SETTLED and not r.motion_isolated)} 条未确认旧目标，运动保持锁定；等待终态确认或显式放弃+隔离'
        if self._state == 'old_run_isolated':
            return '旧目标终态未知（isolated_unknown，cancelConfirmed=false），但运动源已隔离；允许准备新轮，需就绪检查+人工显式启用'
        return '无未确认旧目标；新任务需正常 prepare 与人工显式启用，不会自动续跑'

    def snapshot(self):
        return {'state': self._state, 'barrier': self._barrier,
                'cancelConfirmed': self._cancel_confirmed,
                'motionIsolated': any(rec.motion_isolated for rec in self._records.values()) or self._isolated_release,
                'auditGap': self._audit_gap,
                'evidencePersisted': self._evidence_persisted,
                'journal': self._journal.snapshot() if hasattr(self._journal, 'snapshot') else {},
                'journalError': self._journal_error,
                'pendingOld': [rec.describe() for rec in self._records.values()
                               if rec.state not in _SETTLED and not rec.motion_isolated],
                'isolatedUnknown': [rec.describe() for rec in self._records.values() if rec.motion_isolated],
                'settledCount': sum(1 for rec in self._records.values() if rec.state == 'terminal_confirmed'),
                'abandon': self._abandon, 'currentIdentity': self._current_identity,
                'reason': self._reason(), 'allowedOps': self._allowed_ops()}

    # ---- internals ----

    def _evidence(self, rec, kind, data):
        """Best-effort evidence write; a failure never blocks the RPC path."""
        try:
            self._journal.event(rec.uuid, kind, data)
        except Exception as error:
            self._audit_gap = True
            self._append_audit_gap({'kind': 'evidence_write_failed', 'goalUUID': rec.uuid,
                                    'evidenceKind': kind, 'error': str(error)})
            self._record_cb('recovery_evidence_failed', str(error), goalUUID=rec.uuid, evidenceKind=kind)