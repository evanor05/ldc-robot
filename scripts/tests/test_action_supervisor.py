import concurrent.futures
import types
import unittest
from unittest.mock import Mock

from scripts.action_supervisor import ActionSupervisor


class Handle:
    accepted = True

    def __init__(self):
        self.result = concurrent.futures.Future()
        self.cancel_reply = concurrent.futures.Future()
        self.cancel_goal_async = Mock(return_value=self.cancel_reply)

    def get_result_async(self):
        return self.result


class SupervisionTests(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.before_send = Mock()
        self.supervisor = ActionSupervisor(lambda *args, **kwargs: self.events.append((args, kwargs)), self.before_send)
        self.pending = concurrent.futures.Future()
        self.client = Mock()
        self.client.send_goal_async.return_value = self.pending
        self.accepted = Mock()
        self.completed = Mock()

    def send(self):
        self.supervisor.send(self.client, object(), 'epoch-goal', Mock(), self.accepted, self.completed,
                             goal_uuid=types.SimpleNamespace(uuid=bytes(range(16))))

    def test_revoked_late_acceptance_is_canceled_and_waits_for_terminal(self):
        self.send()
        self.supervisor.revoke_all()
        self.assertFalse(self.supervisor.quiet)
        with self.assertRaises(ValueError):
            self.send()
        handle = Handle()
        handle.goal_id = types.SimpleNamespace(uuid=list(range(16)))
        self.pending.set_result(handle)
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(self.supervisor.snapshot()['pending'][0]['rosGoalId'], bytes(range(16)).hex())
        handle.cancel_reply.set_result(types.SimpleNamespace(return_code=0))
        self.assertFalse(self.supervisor.quiet, 'A cancellation ACK is not a terminal result')
        handle.result.set_result(types.SimpleNamespace(status=5))
        self.assertTrue(self.supervisor.quiet)
        self.completed.assert_called_once()

    def test_rejected_goal_is_proven_not_running(self):
        self.send()
        self.pending.set_result(types.SimpleNamespace(accepted=False))
        self.assertTrue(self.supervisor.quiet)
        self.assertEqual(self.supervisor.snapshot()['history'][-1]['outcome'], 'rejected')

    def test_cancel_rejection_preserves_barrier_and_can_be_explicitly_retried(self):
        self.send()
        handle = Handle()
        self.pending.set_result(handle)
        self.supervisor.revoke_all()
        handle.cancel_reply.set_result(types.SimpleNamespace(return_code=1))
        self.assertFalse(self.supervisor.quiet)
        with self.assertRaises(ValueError):
            self.supervisor.require_quiet()
        handle.cancel_reply = concurrent.futures.Future()
        handle.cancel_goal_async.return_value = handle.cancel_reply
        self.supervisor.revoke_all()
        self.assertEqual(handle.cancel_goal_async.call_count, 2)

    def test_acceptance_error_is_not_evidence_of_remote_termination(self):
        self.send()
        self.pending.set_exception(OSError('lost response'))
        self.assertFalse(self.supervisor.quiet)
        self.assertIn('未确认', self.supervisor.snapshot()['pending'][0]['error'])

    def test_goal_uuid_is_known_before_acceptance_and_survives_lost_response(self):
        self.send()
        before = self.supervisor.snapshot()['pending'][0]['rosGoalId']
        self.assertIsNotNone(before, 'The request must be queryable even without an acceptance reply')
        self.assertRegex(before, r'^[a-f0-9]{32}$')
        request_id = self.client.send_goal_async.call_args.kwargs['goal_uuid']
        self.assertEqual(bytes(request_id.uuid).hex(), before)
        self.pending.set_exception(OSError('acceptance reply lost'))
        self.assertEqual(self.supervisor.snapshot()['pending'][0]['rosGoalId'], before)
        self.assertFalse(self.supervisor.quiet)

    def test_invalid_goal_uuid_is_rejected_before_sending_or_reserving(self):
        for value in [None, types.SimpleNamespace(uuid=b''), types.SimpleNamespace(uuid=bytes(16)),
                      types.SimpleNamespace(uuid=bytes(15)), types.SimpleNamespace(uuid=bytes(17))]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.supervisor.send(self.client, object(), 'bad', Mock(), self.accepted, self.completed,
                                         goal_uuid=value)
                self.assertTrue(self.supervisor.quiet)
                self.client.send_goal_async.assert_not_called()

    def test_preparation_failure_prevents_rpc_and_releases_only_unsent_reservation(self):
        self.before_send.side_effect = OSError('durable store unavailable')
        with self.assertRaises(OSError):
            self.send()
        self.client.send_goal_async.assert_not_called()
        self.assertTrue(self.supervisor.quiet)

    def test_terminal_persistence_failure_retains_barrier_until_explicit_retry(self):
        record = self.supervisor._record
        def fail_terminal(kind, *args, **kwargs):
            if kind == 'action_terminal':
                raise OSError('terminal store unavailable')
            return record(kind, *args, **kwargs)
        self.supervisor._record = fail_terminal
        self.send()
        handle = Handle()
        self.pending.set_result(handle)
        handle.result.set_result(types.SimpleNamespace(status=5))
        self.assertFalse(self.supervisor.quiet)
        self.assertEqual(self.supervisor.snapshot()['pending'][0]['remoteOutcome'], 'canceled')
        self.completed.assert_called_once()
        self.supervisor._record = record
        self.supervisor.revoke_all()
        self.assertTrue(self.supervisor.quiet)

    def test_result_subscription_failure_notifies_owner_and_requests_cancellation(self):
        self.send()
        handle = Handle()
        handle.get_result_async = Mock(side_effect=OSError('result service unavailable'))
        self.pending.set_result(handle)
        self.assertFalse(self.supervisor.quiet)
        handle.cancel_goal_async.assert_called_once()
        self.completed.assert_called_once()
        failed, token = self.completed.call_args.args
        self.assertEqual(token, 'epoch-goal')
        with self.assertRaises(OSError):
            failed.result()

    def test_result_error_or_nonterminal_status_never_settles_the_goal(self):
        for status in [None, 2]:
            with self.subTest(status=status):
                self.setUp()
                self.send()
                handle = Handle()
                self.pending.set_result(handle)
                if status is None:
                    handle.result.set_exception(OSError('lost result'))
                else:
                    handle.result.set_result(types.SimpleNamespace(status=status))
                self.assertFalse(self.supervisor.quiet)
                handle.cancel_goal_async.assert_called_once()

    def test_history_is_bounded_and_success_allows_next_goal(self):
        for i in range(40):
            self.pending = concurrent.futures.Future()
            self.client.send_goal_async.return_value = self.pending
            self.send()
            handle = Handle()
            self.pending.set_result(handle)
            handle.result.set_result(types.SimpleNamespace(status=4))
        self.assertTrue(self.supervisor.quiet)
        self.assertEqual(len(self.supervisor.snapshot()['history']), 32)
