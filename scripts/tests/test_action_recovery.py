"""Recovery gate unit tests: four crash points via mock transport, no ROS.

Covers COORDINATION.md S1 Q2/Q3, the task-3 delivery gate at mock level, and
the adjudication-20261118-b-contract.md C-side hard conditions (frozen coverage
set + watermark, full evidence persistence, isolated persist-failure semantics,
epoch/runEpoch mapping, fail-closed tick, allOldRuns coverage set).
Real Nav2 crash replay happens after the B interface lands (Domain193/197).
"""
from concurrent.futures import Future
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import time
import types
import unittest
import uuid

from scripts.action_journal import ActionJournal
from scripts.action_recovery import (
    REQUEST_TIMEOUT, RecoveryGate, TERMINAL_BY_STATUS, _evidence_error,
)

OLD_PRIVATE = '/ldc/nav_b1r1/navigate_to_pose'
ROOT = '/navigate_to_pose'


def meta(action=OLD_PRIVATE, boot='boot-1', epoch='epoch-1', **extra):
    data = {'token': uuid.uuid4().hex, 'bootId': boot, 'runEpoch': epoch,
            'actionName': action, 'velocityTopic': f'/ldc/velocity/b{boot}/r{epoch}',
            'mapPath': '/tmp/map.yaml', 'target': {'x': 1.0, 'y': 2.0}}
    data.update(extra)
    return data


def cancel_reply(code, uuids=()):
    return types.SimpleNamespace(
        return_code=code,
        goals_canceling=[types.SimpleNamespace(goal_id=types.SimpleNamespace(uuid=bytes.fromhex(u)))
                         for u in uuids])


def evidence(boot='boot-1', epoch='epoch-1', **extra):
    now = time.time()
    data = {'bootId': boot, 'runEpoch': epoch, 'namespace': '/ldc/nav_b1r1',
            'bridgeBootId': 'boot-2', 'bridgeLockPath': '/run/ldc/domain193/bridge.lock',
            'domain': 193, 'abandonSessionId': 'session-1', 'abandonRequestId': '',
            'velocityChain': [{'hop': 'old_controller/cmd_vel_nav', 'to': 'old_smoother'},
                              {'hop': 'old_smoother/output', 'to': 'deprecated_chain'}],
            'oldProcesses': [{'pid': 4242, 'start': '900000', 'disposition': 'terminated',
                              'exitStatus': {'code': 0, 'signal': 'SIGTERM'},
                              'identityVerified': True, 'descendants': []}],
            'graphVerified': True, 'graphCollectedAt': now, 'graphEndpoints': [],
            'oldVelocityPathToFinalExit': False,
            'uniqueFinalPublisher': {'node': 'console_navigation_bridge', 'pid': 5555, 'bootId': 'boot-2'},
            'oldServerDestroyedAt': now, 'verifiedAt': now,
            'verifiedBy': 'nav-runtime'}
    data.update(extra)
    return data


class FakeTransport:
    """Scripted GetResult/CancelGoal. No send_goal capability exists at all.

    Script entries per (action_name, uuid): int status / cancel_reply / 'pending'
    (incomplete future kept in self.pending) / Exception instance (call raises).
    The last entry repeats once the script is exhausted; missing script raises.
    """

    def __init__(self):
        self.result_script = {}
        self.cancel_script = {}
        self.calls = []
        self.pending = []

    def _next(self, script, key):
        steps = script.get(key)
        if not steps:
            raise RuntimeError(f'服务不存在: {key[0]}')
        step = steps[0] if len(steps) == 1 else steps.pop(0)
        if isinstance(step, Exception):
            raise step
        future = Future()
        if step == 'pending':
            self.pending.append(future)
        elif isinstance(step, int):
            future.set_result(types.SimpleNamespace(status=step))
        else:
            future.set_result(step)
        return future

    def get_result(self, action_name, uuid_hex):
        self.calls.append(('get_result', action_name, uuid_hex))
        return self._next(self.result_script, (action_name, uuid_hex))

    def cancel_goal(self, action_name, uuid_hex):
        self.calls.append(('cancel_goal', action_name, uuid_hex))
        return self._next(self.cancel_script, (action_name, uuid_hex))


class GateTestBase(unittest.TestCase):
    def setUp(self):
        self.now = [1000.0]
        self.events = []
        self.journal = ActionJournal(':memory:')
        self.addCleanup(self.journal.close)
        self.transport = FakeTransport()

    def gate(self, journal=None):
        gate = RecoveryGate(journal if journal is not None else self.journal,
                            record=lambda kind, reason='', **kw: self.events.append((kind, reason, kw)),
                            clock=lambda: self.now[0])
        gate.attach_transport(self.transport)
        return gate

    def prepare(self, action=OLD_PRIVATE, boot='boot-1', epoch='epoch-1', journal=None, **extra):
        identifier = uuid.uuid4().hex
        (journal if journal is not None else self.journal).prepare(
            identifier, meta(action, boot, epoch, **extra))
        return identifier

    def script(self, identifier, results=(), cancels=(), action=OLD_PRIVATE):
        if results:
            self.transport.result_script[(action, identifier)] = list(results)
        if cancels:
            self.transport.cancel_script[(action, identifier)] = list(cancels)

    def advance(self, seconds):
        self.now[0] += seconds

    def drive(self, gate, predicate, max_ticks=40):
        """Tick with clock advances until predicate(snapshot) or budget out."""
        snap = gate.snapshot()
        for _ in range(max_ticks):
            if predicate(snap):
                return snap
            self.advance(REQUEST_TIMEOUT + 1)
            snap = gate.tick()
        return snap


class LoadAndBarrierTests(GateTestBase):
    def test_empty_journal_loads_clean_without_barrier(self):
        gate = self.gate()
        snap = gate.load()
        self.assertEqual(snap['state'], 'clean_idle')
        self.assertFalse(snap['barrier'])
        self.assertTrue(snap['cancelConfirmed'])
        self.assertTrue(gate.prepare_allowed)
        self.assertEqual(snap['pendingOld'], [])

    def test_pending_load_establishes_barrier_before_transport_and_enable(self):
        identifier = self.prepare()
        gate = RecoveryGate(self.journal, clock=lambda: self.now[0])  # no transport yet
        snap = gate.load()
        self.assertEqual(snap['state'], 'recovery_barrier')
        self.assertTrue(gate.barrier)
        self.assertFalse(gate.motion_allowed)
        self.assertFalse(gate.prepare_allowed)
        self.assertFalse(snap['cancelConfirmed'])
        self.assertEqual(snap['pendingOld'][0]['goalUUID'], identifier)
        self.assertEqual(snap['pendingOld'][0]['actionName'], OLD_PRIVATE)
        self.assertIn('abandon_old_run', snap['allowedOps'])
        self.assertNotIn('enable', snap['allowedOps'])
        gate.attach_transport(self.transport)  # barrier already established
        self.assertTrue(gate.barrier)

    def test_unloaded_gate_fails_closed(self):
        gate = self.gate()
        self.assertTrue(gate.barrier)
        self.assertFalse(gate.prepare_allowed)
        gate.tick()
        self.assertTrue(gate.barrier)

    def test_legacy_root_and_private_records_keep_their_recorded_names(self):
        legacy = self.prepare(action=ROOT)
        private = self.prepare(action=OLD_PRIVATE)
        gate = self.gate()
        gate.set_current_identity('boot-2', 'epoch-2', ROOT)
        snap = gate.load()
        by_uuid = {row['goalUUID']: row for row in snap['pendingOld']}
        self.assertEqual(by_uuid[legacy]['actionName'], ROOT)
        self.assertTrue(by_uuid[legacy]['nameCollisionRisk'])
        self.assertEqual(by_uuid[private]['actionName'], OLD_PRIVATE)
        self.assertFalse(by_uuid[private]['nameCollisionRisk'])

    def test_epoch_runepoch_adapter_same_generation_no_collision(self):
        # B begin uses 'epoch'; journal metadata uses 'runEpoch' (item ④).
        identifier = self.prepare(action=ROOT, boot='boot-1', epoch='epoch-1')
        gate = self.gate()
        gate.set_current_identity('boot-1', 'epoch-1', ROOT)
        snap = gate.load()
        row = snap['pendingOld'][0]
        self.assertEqual(row['goalUUID'], identifier)
        self.assertEqual(row['runEpoch'], 'epoch-1')
        self.assertFalse(row['nameCollisionRisk'])  # same boot+epoch == same generation
        gate.set_current_identity('boot-1', 'epoch-2', ROOT)
        self.assertTrue(gate.snapshot()['pendingOld'][0]['nameCollisionRisk'])

    def test_missing_action_name_keeps_uncertainty_and_audit_gap(self):
        identifier = uuid.uuid4().hex
        self.journal.prepare(identifier, {'bootId': 'boot-1'})
        gate = self.gate()
        snap = gate.load()
        self.assertTrue(snap['barrier'])
        self.assertTrue(snap['auditGap'])
        self.assertEqual(snap['pendingOld'][0]['subReason'], 'missing_action_name')
        gate.tick()
        self.assertEqual(self.transport.calls, [])  # nowhere to query, never guess


class JournalFaultTests(GateTestBase):
    def setUp(self):
        super().setUp()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'actions.sqlite3'

    def test_foreign_database_is_rejected_and_not_modified(self):
        conn = sqlite3.connect(self.path)
        conn.execute('CREATE TABLE foreign_data(x)')
        conn.execute("INSERT INTO foreign_data VALUES('keep me')")
        conn.commit()
        conn.close()
        before = self.path.read_bytes()
        gate = self.gate(ActionJournal(self.path))
        snap = gate.load()
        self.assertEqual(snap['state'], 'journal_fault')
        self.assertTrue(gate.barrier)
        self.assertFalse(gate.prepare_allowed)
        self.assertTrue(snap['auditGap'])
        self.assertEqual(self.path.read_bytes(), before)

    def test_corrupt_metadata_is_a_fault_not_a_clean_slate(self):
        journal = ActionJournal(self.path)
        identifier = uuid.uuid4().hex
        journal.prepare(identifier, meta())
        journal.close()
        conn = sqlite3.connect(self.path)
        conn.execute("UPDATE goals SET metadata='{broken json'")
        conn.commit()
        conn.close()
        gate = self.gate(ActionJournal(self.path))
        snap = gate.load()
        self.assertEqual(snap['state'], 'journal_fault')
        self.assertTrue(gate.barrier)
        self.assertFalse(snap['cancelConfirmed'])

    def test_readonly_journal_faults_closed(self):
        journal = ActionJournal(self.path)
        identifier = uuid.uuid4().hex
        journal.prepare(identifier, meta())
        journal.close()
        self.path.chmod(0o400)
        Path(self.temp.name).chmod(0o500)
        self.addCleanup(lambda: (Path(self.temp.name).chmod(0o700), self.path.chmod(0o600)))
        gate = self.gate(ActionJournal(self.path))
        snap = gate.load()
        self.assertEqual(snap['state'], 'journal_fault')
        self.assertTrue(gate.barrier)
        self.assertFalse(gate.prepare_allowed)

    def test_fault_abandon_with_all_old_runs_evidence_releases_with_audit_gap(self):
        conn = sqlite3.connect(self.path)
        conn.execute('CREATE TABLE foreign_data(x)')
        conn.commit()
        conn.close()
        gate = self.gate(ActionJournal(self.path))
        gate.load()
        gate.abandon_old_run('session-1', 'req-1')
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence())  # needs allOldRuns+coveredRuns
        snap = gate.submit_isolation_evidence(evidence(
            allOldRuns=True, coveredRuns=[{'bootId': 'boot-1', 'runEpoch': 'epoch-1'}],
            abandonRequestId='req-1'))
        self.assertEqual(snap['state'], 'old_run_isolated')
        self.assertFalse(snap['barrier'])
        self.assertFalse(snap['cancelConfirmed'])
        self.assertTrue(snap['motionIsolated'])
        self.assertTrue(snap['auditGap'])
        self.assertFalse(gate.prepare_allowed)  # journal still broken: repair first
        self.assertIn('repair_journal', snap['allowedOps'])

    def test_fault_release_survives_journal_repair_within_coverage_only(self):
        journal = ActionJournal(self.path)
        covered = self.prepare(journal=journal)
        journal.close()
        conn = sqlite3.connect(self.path)
        conn.execute("UPDATE goals SET metadata='{broken'")
        conn.commit()
        conn.close()
        gate = self.gate(ActionJournal(self.path))
        gate.load()
        gate.abandon_old_run('session-1')
        gate.submit_isolation_evidence(evidence(
            allOldRuns=True, coveredRuns=[{'bootId': 'boot-1', 'runEpoch': 'epoch-1'}]))
        # Repair metadata externally; the covered row reappears as isolated.
        conn = sqlite3.connect(self.path)
        conn.execute('UPDATE goals SET metadata=?', (json.dumps(meta()),))
        conn.commit()
        conn.close()
        snap = gate.load()
        self.assertFalse(snap['barrier'])
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], covered)
        self.assertFalse(snap['cancelConfirmed'])
        self.assertEqual(self.transport.calls, [])
        # A row OUTSIDE the coverage set (different epoch) re-raises the barrier.
        conn = sqlite3.connect(self.path)
        outsider = uuid.uuid4().hex
        conn.execute('INSERT INTO goals VALUES(?,?,?,NULL,NULL)',
                     (outsider, time.time(), json.dumps(meta(epoch='epoch-other'))))
        conn.commit()
        conn.close()
        snap = gate.load()
        self.assertTrue(snap['barrier'])
        self.assertIn(outsider, [row['goalUUID'] for row in snap['pendingOld']])
        self.assertFalse(snap['cancelConfirmed'])


class TerminalSettlementTests(GateTestBase):
    def test_terminal_statuses_settle_only_after_persist(self):
        for status, outcome in TERMINAL_BY_STATUS.items():
            with self.subTest(status=status):
                identifier = self.prepare()
                self.script(identifier, results=[status])
                gate = self.gate()
                gate.load()
                snap = self.drive(gate, lambda s: not s['barrier'])
                self.assertEqual(snap['state'], 'recovered_idle')
                self.assertFalse(snap['barrier'])
                self.assertTrue(snap['cancelConfirmed'])
                self.assertEqual(snap['settledCount'], 1)
                self.assertEqual(self.journal.pending(), [])
                row = self.journal.connection.execute(
                    'SELECT outcome FROM goals WHERE uuid=?', (identifier,)).fetchone()
                self.assertEqual(row[0], outcome)

    def test_unknown_status_never_settles_and_tries_uuid_cancel(self):
        identifier = self.prepare()
        self.script(identifier, results=[0], cancels=[cancel_reply(2)])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: any(
            c[0] == 'cancel_goal' for c in self.transport.calls))
        self.assertTrue(snap['barrier'])
        self.assertEqual(snap['pendingOld'][0]['recoveryState'], 'cannot_confirm')
        self.assertIn(snap['pendingOld'][0]['subReason'],
                      ('result_unknown_or_expired', 'cancel_unknown_goal'))
        self.assertEqual(self.journal.pending()[0]['goalUUID'], identifier)
        self.assertIn(('cancel_goal', OLD_PRIVATE, identifier), self.transport.calls)

    def test_executing_status_requests_cancel_then_waits_terminal(self):
        identifier = self.prepare()
        self.script(identifier, results=[2, 2, 5], cancels=[cancel_reply(0, [identifier])])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: s['pendingOld'] and
                          s['pendingOld'][0]['recoveryState'] == 'cancel_acknowledged')
        self.assertEqual(snap['pendingOld'][0]['recoveryState'], 'cancel_acknowledged')
        self.assertTrue(snap['barrier'])  # ACK is NOT terminal
        self.assertFalse(snap['cancelConfirmed'])
        self.assertEqual(len(self.journal.pending()), 1)
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])
        self.assertTrue(snap['cancelConfirmed'])
        self.assertEqual(self.journal.pending(), [])

    def test_cancel_ack_empty_rejected_unknown_terminated_keep_barrier(self):
        cases = [(cancel_reply(0, []), 'cancel_ack_empty_list'),
                 (cancel_reply(1), 'cancel_rejected'),
                 (cancel_reply(2), 'cancel_unknown_goal'),
                 (cancel_reply(3), 'cancel_goal_terminated')]
        for reply, reason in cases:
            with self.subTest(reason=reason):
                identifier = self.prepare()
                self.script(identifier, results=[0, 0], cancels=[reply])
                gate = self.gate()
                gate.load()
                snap = self.drive(gate, lambda s: any(
                    row['goalUUID'] == identifier and row['subReason'] == reason
                    for row in s['pendingOld']))
                self.assertTrue(snap['barrier'])
                row = next(r for r in snap['pendingOld'] if r['goalUUID'] == identifier)
                self.assertEqual(row['subReason'], reason)
                self.assertIn(identifier, [r['goalUUID'] for r in self.journal.pending()])

    def test_code3_followed_by_expired_cache_stays_unconfirmed(self):
        identifier = self.prepare()
        self.script(identifier, results=[0, 0], cancels=[cancel_reply(3)])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: s['pendingOld'] and
                          s['pendingOld'][0]['subReason'] == 'result_unknown_or_expired')
        self.assertTrue(snap['barrier'])
        self.assertEqual(snap['pendingOld'][0]['subReason'], 'result_unknown_or_expired')

    def test_result_timeout_and_service_missing_keep_barrier(self):
        identifier = self.prepare()
        self.script(identifier, results=['pending'])
        gate = self.gate()
        gate.load()
        gate.tick()
        self.advance(REQUEST_TIMEOUT + 1)
        snap = gate.tick()
        by_uuid = {row['goalUUID']: row for row in snap['pendingOld']}
        self.assertEqual(by_uuid[identifier]['subReason'], 'result_timeout')
        self.assertTrue(snap['barrier'])
        other = self.prepare()
        self.script(other, results=[RuntimeError('服务不存在')])
        gate.load()  # idempotent merge picks up the new row
        snap = self.drive(gate, lambda s: any(
            row['goalUUID'] == other and row['subReason'] == 'service_missing'
            for row in s['pendingOld']))
        by_uuid = {row['goalUUID']: row for row in snap['pendingOld']}
        self.assertEqual(by_uuid[other]['subReason'], 'service_missing')
        self.assertTrue(snap['barrier'])

    def test_late_terminal_after_timeout_still_settles(self):
        identifier = self.prepare()
        self.script(identifier, results=['pending'])
        gate = self.gate()
        gate.load()
        gate.tick()
        self.advance(REQUEST_TIMEOUT + 1)
        gate.tick()  # declared timeout; future kept for late completion
        self.transport.pending[0].set_result(types.SimpleNamespace(status=5))
        snap = gate.tick()  # late response is still valid evidence
        self.assertFalse(snap['barrier'])
        self.assertEqual(self.journal.pending(), [])

    def test_backoff_limits_query_rate(self):
        identifier = self.prepare()
        self.script(identifier, results=[0], cancels=[cancel_reply(2)])
        gate = self.gate()
        gate.load()
        for _ in range(5):
            gate.tick()
        queries = [c for c in self.transport.calls if c[0] == 'get_result']
        self.assertEqual(len(queries), 1)  # backoff holds until clock advances
        self.advance(60)
        gate.tick()
        queries = [c for c in self.transport.calls if c[0] == 'get_result']
        self.assertEqual(len(queries), 2)

    def test_name_collision_terminal_for_exact_uuid_still_settles(self):
        identifier = self.prepare(action=ROOT)
        self.script(identifier, results=[5], action=ROOT)
        gate = self.gate()
        gate.set_current_identity('boot-2', 'epoch-2', ROOT)
        gate.load()
        self.assertTrue(gate.snapshot()['pendingOld'][0]['nameCollisionRisk'])
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])  # UUID binds identity; new instance cannot know u1
        self.assertEqual(self.journal.pending(), [])

    def test_tick_internal_error_is_fail_closed(self):
        identifier = self.prepare()
        gate = self.gate()
        gate.load()

        def broken(rec, now):
            raise RuntimeError('内部异常')

        gate._drive_record = broken
        snap = gate.tick()
        self.assertTrue(snap['barrier'])  # never released by an internal error
        self.assertFalse(snap['cancelConfirmed'])
        self.assertTrue(any(e[0] == 'recovery_tick_error' for e in self.events))
        self.assertEqual(len(self.journal.pending()), 1)


class PersistFailureTests(GateTestBase):
    def test_terminal_write_failure_keeps_barrier_and_known_outcome(self):
        identifier = self.prepare()
        self.script(identifier, results=[5])
        gate = self.gate()
        gate.load()
        original = self.journal.terminal

        def broken(uuid_arg, outcome):
            raise sqlite3.OperationalError('attempt to write a readonly database')

        self.journal.terminal = broken
        snap = self.drive(gate, lambda s: s['pendingOld'] and
                          s['pendingOld'][0]['recoveryState'] == 'terminal_persist_failed')
        self.assertTrue(snap['barrier'])
        row = snap['pendingOld'][0]
        self.assertEqual(row['recoveryState'], 'terminal_persist_failed')
        self.assertEqual(row['knownOutcome'], 'canceled')
        self.assertFalse(snap['cancelConfirmed'])
        self.assertIn('retry_persist', snap['allowedOps'])
        self.journal.terminal = original
        snap = gate.retry_persist(identifier)  # explicit retry only
        self.assertFalse(snap['barrier'])
        self.assertTrue(snap['cancelConfirmed'])
        self.assertEqual(self.journal.pending(), [])

    def test_retry_persist_requires_pending_failed_record(self):
        gate = self.gate()
        gate.load()
        with self.assertRaises(ValueError):
            gate.retry_persist(uuid.uuid4().hex)

    def test_evidence_write_failure_never_blocks_settlement(self):
        identifier = self.prepare()
        self.script(identifier, results=[6])
        gate = self.gate()
        gate.load()

        def broken_event(*args, **kwargs):
            raise sqlite3.OperationalError('disk I/O error')

        self.journal.event = broken_event
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])  # terminal write succeeded; evidence is best-effort
        self.assertTrue(snap['auditGap'])
        self.assertEqual(self.journal.pending(), [])


class CrashPointTests(GateTestBase):
    """The four delivery-gate crash points, constructed at mock level."""

    def test_cp1_uuid_persisted_rpc_never_sent_no_redispatch(self):
        identifier = self.prepare()  # journal row only; no action_sent evidence
        self.script(identifier, results=[0], cancels=[cancel_reply(2)])
        gate = self.gate()
        gate.load()
        for _ in range(3):
            self.advance(60)
            snap = gate.tick()
        self.assertTrue(snap['barrier'])
        self.assertIn(snap['pendingOld'][0]['subReason'],
                      ('result_unknown_or_expired', 'cancel_unknown_goal'))
        # Every call is UUID-targeted query/cancel on the recorded name; the
        # transport has no send_goal capability and zero UUID never appears.
        self.assertTrue(self.transport.calls)
        for kind, action, uuid_hex in self.transport.calls:
            self.assertIn(kind, ('get_result', 'cancel_goal'))
            self.assertEqual(action, OLD_PRIVATE)
            self.assertEqual(uuid_hex, identifier)
        self.assertFalse(hasattr(self.transport, 'send_goal'))
        self.assertEqual(len(self.journal.pending()), 1)  # never auto-deleted
        # Only exit: explicit abandonment + verified isolation.
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])
        self.assertFalse(snap['cancelConfirmed'])
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], identifier)

    def test_cp2_sent_acceptance_lost_settles_via_query_and_cancel(self):
        identifier = self.prepare()
        self.journal.event(identifier, 'action_sent', {'reason': '发送请求已提交'})
        self.script(identifier, results=[2, 5], cancels=[cancel_reply(0, [identifier])])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])
        self.assertEqual(self.journal.pending(), [])
        row = self.journal.connection.execute(
            'SELECT outcome FROM goals WHERE uuid=?', (identifier,)).fetchone()
        self.assertEqual(row[0], 'canceled')

    def test_cp2_missing_sent_evidence_is_not_proof_of_unsent(self):
        identifier = self.prepare()  # evidence write may have died with the crash
        self.script(identifier, results=[1, 5], cancels=[cancel_reply(0, [identifier])])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: not s['barrier'])
        # status 1 (accepted) proves it WAS sent despite missing evidence
        self.assertFalse(snap['barrier'])
        self.assertEqual(self.journal.pending(), [])

    def test_cp3_executing_cancel_ack_then_terminal(self):
        identifier = self.prepare()
        self.journal.event(identifier, 'action_sent', {'reason': 'sent'})
        self.script(identifier, results=[2, 3, 5], cancels=[cancel_reply(0, [identifier])])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: s['pendingOld'] and
                          s['pendingOld'][0]['recoveryState'] == 'cancel_acknowledged')
        self.assertTrue(snap['barrier'])  # ACK: still barrier
        self.assertEqual(snap['pendingOld'][0]['recoveryState'], 'cancel_acknowledged')
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])
        self.assertEqual(self.journal.pending(), [])

    def test_cp4_terminal_received_before_persist_cache_alive(self):
        identifier = self.prepare()  # in-memory terminal died with the crash
        self.script(identifier, results=[4])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: not s['barrier'])
        self.assertFalse(snap['barrier'])
        row = self.journal.connection.execute(
            'SELECT outcome FROM goals WHERE uuid=?', (identifier,)).fetchone()
        self.assertEqual(row[0], 'succeeded')

    def test_cp4_terminal_received_before_persist_cache_expired(self):
        identifier = self.prepare()
        self.script(identifier, results=[0], cancels=[cancel_reply(3)])
        gate = self.gate()
        gate.load()
        gate.tick()
        snap = gate.tick()
        self.assertTrue(snap['barrier'])  # never fabricate the remembered terminal
        self.assertEqual(len(self.journal.pending()), 1)
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])
        self.assertFalse(snap['cancelConfirmed'])  # stays false while isolated
        self.assertTrue(snap['motionIsolated'])
        self.assertEqual(len(self.journal.pending()), 1)  # row kept for cancel tracking


class AbandonAndIsolationTests(GateTestBase):
    def test_abandon_requires_explicit_session_and_pending(self):
        gate = self.gate()
        gate.load()
        with self.assertRaises(ValueError):
            gate.abandon_old_run('')
        with self.assertRaises(ValueError):
            gate.abandon_old_run('session-1')  # clean idle: nothing to abandon
        self.prepare()
        gate.load()
        with self.assertRaises(ValueError):
            gate.abandon_old_run('')

    def test_abandon_sequence_waits_for_evidence(self):
        identifier = self.prepare()
        self.script(identifier, results=[0], cancels=[cancel_reply(2)])
        gate = self.gate()
        gate.load()
        gate.tick()
        snap = gate.abandon_old_run('session-1', 'req-9')
        self.assertEqual(snap['state'], 'abandon_requested')
        self.assertTrue(snap['barrier'])
        self.assertEqual(snap['abandon']['sessionId'], 'session-1')
        self.assertIn('wait_isolation', snap['allowedOps'])
        snap = gate.submit_isolation_evidence(evidence(abandonRequestId='req-9'))
        self.assertEqual(snap['state'], 'old_run_isolated')
        self.assertFalse(snap['barrier'])
        self.assertTrue(gate.prepare_allowed)
        self.assertFalse(snap['cancelConfirmed'])
        self.assertTrue(snap['motionIsolated'])
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], identifier)
        self.assertEqual(len(self.journal.pending()), 1)  # never cleared to resume
        kinds = [row[2] for row in self.journal.connection.execute(
            'SELECT uuid,at,kind FROM evidence WHERE uuid=?', (identifier,))]
        self.assertIn('recovery_isolated', kinds)

    def test_invalid_evidence_is_rejected_and_barrier_holds(self):
        self.prepare()
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        bad_packages = [
            evidence(graphVerified=False),
            evidence(graphCollectedAt=None),
            evidence(graphEndpoints=None),
            evidence(oldVelocityPathToFinalExit=True),
            evidence(uniqueFinalPublisher=''),
            evidence(bridgeBootId=''),
            evidence(bridgeLockPath=''),
            evidence(domain=None),
            evidence(abandonSessionId='other-session'),
            evidence(velocityChain=[]),
            evidence(oldProcesses=[]),
            evidence(oldProcesses=[{'pid': 1, 'disposition': 'running'}]),
            evidence(oldProcesses=[{'pid': 1, 'disposition': 'terminated'}]),
            evidence(oldProcesses=[{'pid': 1, 'startTicks': 5, 'disposition': 'terminated',
                                    'exitStatus': 0}]),  # no identityVerified
            evidence(verifiedAt=None),
            evidence(verifiedAt=time.time() - 1000),  # stale
            evidence(bootId=None, runEpoch=None),
            evidence(allOldRuns=True),  # wildcard without coverage set
            evidence(allOldRuns=True, coveredRuns=[{'bootId': 'b'}]),
            'not-a-dict',
        ]
        for package in bad_packages:
            with self.subTest(package=package):
                with self.assertRaises(ValueError):
                    gate.submit_isolation_evidence(package)
                self.assertTrue(gate.barrier)
                self.assertEqual(gate.snapshot()['state'], 'abandon_requested')

    def test_evidence_binding_to_current_bridge_domain_and_run(self):
        self.prepare()
        gate = self.gate()
        gate.set_current_identity('boot-2', 'epoch-2', '/ldc/nav_b2r2/navigate_to_pose', domain=193)
        gate.load()
        gate.abandon_old_run('session-1', 'req-X')
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(abandonRequestId='req-Y'))
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(bridgeBootId='boot-other'))
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(abandonSessionId='other-session'))
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(domain=197))
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(boot='boot-2', epoch='epoch-2'))  # current run
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence(
                allOldRuns=True, coveredRuns=[{'bootId': 'boot-2', 'runEpoch': 'epoch-2'}],
                abandonRequestId='req-X'))
        snap = gate.submit_isolation_evidence(evidence(abandonRequestId='req-X'))
        self.assertFalse(snap['barrier'])

    def test_evidence_without_abandon_is_rejected(self):
        self.prepare()
        gate = self.gate()
        gate.load()
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(evidence())

    def test_evidence_must_cover_the_recorded_generation(self):
        identifier = self.prepare(boot='boot-1', epoch='epoch-1')
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence(boot='boot-other', epoch='epoch-other'))
        self.assertTrue(snap['barrier'])  # nothing covered
        self.assertEqual(snap['state'], 'abandon_requested')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], identifier)

    def test_new_goal_after_evidence_is_not_released_by_old_coverage(self):
        # adjudication item ①: current-boot and post-watermark goals stay barred.
        old = self.prepare()
        gate = self.gate()
        gate.set_current_identity('boot-2', 'epoch-2', '/ldc/nav_b2r2/navigate_to_pose')
        gate.load()
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])
        # New dispatch on the CURRENT boot after the evidence watermark.
        current = self.prepare(boot='boot-2', epoch='epoch-2',
                               action='/ldc/nav_b2r2/navigate_to_pose')
        snap = gate.load()
        self.assertTrue(snap['barrier'])
        self.assertIn(current, [row['goalUUID'] for row in snap['pendingOld']])
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], old)
        self.assertFalse(gate.prepare_allowed)
        # Even an old-boot row prepared AFTER the watermark is not covered.
        late = self.prepare(boot='boot-1', epoch='epoch-1')
        snap = gate.load()
        self.assertTrue(snap['barrier'])
        self.assertIn(late, [row['goalUUID'] for row in snap['pendingOld']])

    def test_all_old_runs_coverage_set_applies_only_to_listed_runs(self):
        covered = self.prepare(boot='boot-1', epoch='epoch-1')
        outsider = self.prepare(boot='boot-9', epoch='epoch-9')
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence(
            allOldRuns=True, coveredRuns=[{'bootId': 'boot-1', 'runEpoch': 'epoch-1'}]))
        self.assertTrue(snap['barrier'])  # outsider still unconfirmed
        self.assertEqual(snap['isolatedUnknown'][0]['goalUUID'], covered)
        self.assertIn(outsider, [row['goalUUID'] for row in snap['pendingOld']])

    def test_cancel_abandon_before_and_after_evidence(self):
        self.prepare()
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        snap = gate.cancel_abandon('session-1')
        self.assertEqual(snap['state'], 'recovery_barrier')
        gate.abandon_old_run('session-1')
        gate.submit_isolation_evidence(evidence())
        with self.assertRaises(ValueError):
            gate.cancel_abandon('session-1')

    def test_duplicate_recovery_is_idempotent(self):
        identifier = self.prepare()
        gate = self.gate()
        gate.load()
        gate.load()
        self.assertEqual(len(gate.snapshot()['pendingOld']), 1)
        gate.abandon_old_run('session-1')
        gate.abandon_old_run('session-1')
        abandons = [e for e in self.events if e[0] == 'recovery_abandon_requested']
        self.assertEqual(len(abandons), 1)
        package = evidence()
        gate.submit_isolation_evidence(package)
        snap = gate.submit_isolation_evidence(package)
        self.assertEqual(len(snap['isolatedUnknown']), 1)
        self.assertEqual(len(self.journal.pending()), 1)
        gate.load()  # reload after isolation keeps classification
        self.assertEqual(gate.snapshot()['isolatedUnknown'][0]['goalUUID'], identifier)
        self.assertFalse(gate.barrier)

    def test_mixed_settled_and_isolated_keeps_cancel_confirmed_false(self):
        settled = self.prepare()
        isolated = self.prepare()
        self.script(settled, results=[5])
        gate = self.gate()
        gate.load()
        snap = self.drive(gate, lambda s: s['settledCount'] == 1)
        self.assertEqual(snap['settledCount'], 1)
        self.assertTrue(snap['barrier'])  # second record still unconfirmed
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])
        self.assertFalse(snap['cancelConfirmed'])
        self.assertTrue(snap['motionIsolated'])
        self.assertEqual(snap['settledCount'], 1)
        self.assertEqual(len(snap['isolatedUnknown']), 1)
        self.assertEqual([row['goalUUID'] for row in self.journal.pending()], [isolated])

    def test_isolated_old_goal_still_settles_if_old_server_reappears(self):
        identifier = self.prepare()
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        gate.submit_isolation_evidence(evidence(oldServerDestroyedAt=None,
                                                oldProcesses=[{'pid': 4242, 'startTicks': 900000,
                                                               'disposition': 'quarantined',
                                                               'namespace': '/ldc/quarantine'}]))
        # Old ledger keeps cancel tracking: a late terminal still settles it.
        self.script(identifier, results=[5])
        snap = self.drive(gate, lambda s: s['cancelConfirmed'])
        row = self.journal.connection.execute(
            'SELECT outcome FROM goals WHERE uuid=?', (identifier,)).fetchone()
        self.assertEqual(row[0], 'canceled')
        self.assertTrue(snap['cancelConfirmed'])
        self.assertTrue(snap['motionIsolated'])  # history kept

    def test_isolated_terminal_persist_failure_does_not_raise_barrier(self):
        # adjudication item ③: motion already isolated; storage failure must
        # not re-lock the current run, and cancelConfirmed stays false.
        identifier = self.prepare()
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        gate.submit_isolation_evidence(evidence())
        self.script(identifier, results=[5])
        original = self.journal.terminal

        def broken(uuid_arg, outcome):
            raise sqlite3.OperationalError('disk full')

        self.journal.terminal = broken
        snap = self.drive(gate, lambda s: s['isolatedUnknown'] and
                          s['isolatedUnknown'][0]['recoveryState'] == 'terminal_persist_failed')
        self.assertFalse(snap['barrier'])  # NOT re-raised
        self.assertTrue(gate.prepare_allowed)  # T15: current-run gate unaffected
        self.assertFalse(snap['cancelConfirmed'])
        self.assertTrue(snap['motionIsolated'])
        self.journal.terminal = original
        snap = gate.retry_persist(identifier)
        self.assertFalse(snap['barrier'])
        self.assertTrue(snap['cancelConfirmed'])


class EvidencePersistenceTests(GateTestBase):
    def setUp(self):
        super().setUp()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'actions.sqlite3'

    def test_full_package_persisted_immutably_with_reference(self):
        journal = ActionJournal(self.path)
        identifier = self.prepare(journal=journal)
        gate = self.gate(journal)
        gate.load()
        gate.abandon_old_run('session-1')
        package = evidence()
        snap = gate.submit_isolation_evidence(package)
        self.assertIs(snap['evidencePersisted'], True)
        files = list(Path(self.temp.name).glob('isolation-evidence-*.json'))
        self.assertEqual(len(files), 1)
        stored = json.loads(files[0].read_text())
        self.assertEqual(stored, package)  # FULL package, not a digest subset
        digest = hashlib.sha256(json.dumps(package, sort_keys=True, ensure_ascii=False,
                                           allow_nan=False).encode()).hexdigest()
        self.assertIn(digest[:16], files[0].name)
        rows = [json.loads(row[0]) for row in journal.connection.execute(
            "SELECT data FROM evidence WHERE uuid=? AND kind='recovery_isolated'", (identifier,))]
        self.assertEqual(rows[0]['evidenceSha256'], digest)
        self.assertEqual(rows[0]['evidenceRef'], str(files[0]))
        self.assertFalse(rows[0]['cancelConfirmed'])
        self.assertTrue(rows[0]['motionIsolated'])
        # Resubmission verifies the existing file instead of overwriting.
        gate.submit_isolation_evidence(package)
        self.assertEqual(len(list(Path(self.temp.name).glob('isolation-evidence-*.json'))), 1)

    def test_tampered_evidence_file_is_rejected_not_overwritten(self):
        journal = ActionJournal(self.path)
        self.prepare(journal=journal)
        gate = self.gate(journal)
        gate.load()
        gate.abandon_old_run('session-1')
        package = evidence()
        gate.submit_isolation_evidence(package)
        target = next(Path(self.temp.name).glob('isolation-evidence-*.json'))
        target.write_text('{"tampered": true}')
        with self.assertRaises(ValueError):
            gate.submit_isolation_evidence(package)
        self.assertEqual(target.read_text(), '{"tampered": true}')  # never overwritten

    def test_unwritable_storage_leaves_persistent_audit_gap_not_false_claim(self):
        journal = ActionJournal(self.path)
        identifier = self.prepare(journal=journal)
        gate = self.gate(journal)
        gate.load()
        gate.abandon_old_run('session-1')
        self.path.chmod(0o400)
        Path(self.temp.name).chmod(0o500)
        self.addCleanup(lambda: (Path(self.temp.name).chmod(0o700), self.path.chmod(0o600)))
        snap = gate.submit_isolation_evidence(evidence())
        self.assertFalse(snap['barrier'])  # verified isolation still releases (decision.md)
        self.assertIs(snap['evidencePersisted'], False)  # no false persistence claim
        self.assertTrue(snap['auditGap'])
        self.assertEqual(list(Path(self.temp.name).glob('isolation-evidence-*.json')), [])
        self.assertTrue(any(e[0] == 'recovery_evidence_unpersisted' for e in self.events))
        # Restart loses the in-memory evidence → fail-closed barrier again.
        gate2 = self.gate(ActionJournal(self.path))
        self.path.chmod(0o600)
        Path(self.temp.name).chmod(0o700)
        snap2 = gate2.load()
        self.assertTrue(snap2['barrier'])
        self.assertEqual(snap2['pendingOld'][0]['goalUUID'], identifier)

    def test_memory_journal_reports_memory_persistence(self):
        self.prepare()
        gate = self.gate()
        gate.load()
        gate.abandon_old_run('session-1')
        snap = gate.submit_isolation_evidence(evidence())
        self.assertEqual(snap['evidencePersisted'], 'memory')


class CapacityAndNoResumeTests(GateTestBase):
    def test_capacity_full_blocks_new_prepare_without_deleting_pending(self):
        journal = ActionJournal(':memory:', pending_limit=2)
        self.addCleanup(journal.close)
        first = self.prepare(journal=journal)
        second = self.prepare(journal=journal)
        gate = self.gate(journal)
        snap = gate.load()
        self.assertTrue(snap['barrier'])
        self.assertEqual(len(snap['pendingOld']), 2)
        with self.assertRaises(ValueError):
            journal.prepare(uuid.uuid4().hex, meta())
        self.assertEqual(len(journal.pending()), 2)  # no auto-delete
        self.assertEqual({row['goalUUID'] for row in journal.pending()}, {first, second})

    def test_duplicate_prepare_same_uuid_does_not_overwrite(self):
        identifier = self.prepare()
        with self.assertRaises(sqlite3.IntegrityError):
            self.journal.prepare(identifier, meta(token='replacement'))
        self.assertEqual(len(self.journal.pending()), 1)

    def test_no_event_redispatches_or_auto_enables(self):
        identifier = self.prepare()
        self.script(identifier, results=[2, 2], cancels=[cancel_reply(0, [identifier])])
        gate = self.gate()
        gate.load()
        for _ in range(4):
            self.advance(60)
            gate.tick()
        # Refresh/restart/sensor-recovery analogues: repeated loads and ticks.
        gate.load()
        gate.tick()
        self.assertTrue(gate.barrier)  # never auto-settles, never auto-resumes
        for kind, action, uuid_hex in self.transport.calls:
            self.assertIn(kind, ('get_result', 'cancel_goal'))
            self.assertEqual(uuid_hex, identifier)
        self.assertEqual(len(self.journal.pending()), 1)

    def test_snapshot_answers_the_three_page_questions(self):
        identifier = self.prepare()
        self.script(identifier, results=[0], cancels=[cancel_reply(2)])
        gate = self.gate()
        gate.load()
        gate.tick()
        snap = gate.snapshot()
        self.assertEqual(snap['pendingOld'][0]['goalUUID'], identifier)  # 哪个旧目标未结清
        self.assertIn('未确认旧目标', snap['reason'])                     # 为什么禁止运动
        self.assertIn('abandon_old_run', snap['allowedOps'])             # 现在能做什么
        self.assertIn('estop', snap['allowedOps'])


class EvidenceSchemaTests(unittest.TestCase):
    def test_minimal_valid_package_passes(self):
        self.assertEqual(_evidence_error(evidence()), '')
        self.assertEqual(_evidence_error(evidence(
            allOldRuns=True, bootId=None, runEpoch=None,
            coveredRuns=[{'bootId': 'boot-1', 'runEpoch': 'epoch-1'}])), '')

    def test_quarantined_processes_need_no_exit_status(self):
        package = evidence(oldProcesses=[{'pid': 7, 'startTicks': 100,
                                          'disposition': 'quarantined',
                                          'namespace': '/ldc/quarantine'}])
        self.assertEqual(_evidence_error(package), '')

    def test_process_gone_natural_death_needs_no_exit_status(self):
        package = evidence(oldProcesses=[{'pid': 7, 'disposition': 'process_gone'}],
                           oldServerDestroyedAt=time.time())
        self.assertEqual(_evidence_error(package), '')

    def test_terminated_requires_identity_verification_fail_closed(self):
        base = {'pid': 7, 'startTicks': 100, 'disposition': 'terminated', 'exitStatus': 0}
        self.assertNotEqual(_evidence_error(evidence(oldProcesses=[base])), '')
        self.assertNotEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'identityVerified': True}])), '')  # no descendants info
        self.assertEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'identityVerified': True, 'descendants': []}])), '')
        # canonical field name is 'start' (/proc/stat field22); startTicks is a
        # transitional alias (B interface doc §B.3).
        canonical = {'pid': 7, 'start': '100', 'disposition': 'terminated',
                     'exitStatus': {'code': 0, 'signal': 'SIGTERM'},
                     'identityVerified': True, 'descendants': []}
        self.assertEqual(_evidence_error(evidence(oldProcesses=[canonical])), '')
        self.assertEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'identityVerified': True,
                           'descendantsEvidenceRef': '/log/isolation-evidence-x.json#descendants'}])), '')
        self.assertNotEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'identityVerified': False, 'descendants': []}])), '')
        no_ticks = {k: v for k, v in base.items() if k != 'startTicks'}
        self.assertNotEqual(_evidence_error(evidence(
            oldProcesses=[{**no_ticks, 'identityVerified': True, 'descendants': []}])), '')

    def test_descendants_must_be_disposed_not_left_running(self):
        base = {'pid': 7, 'startTicks': 100, 'disposition': 'terminated', 'exitStatus': 0,
                'identityVerified': True}
        running_child = {'pid': 8, 'disposition': 'running'}
        self.assertNotEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'descendants': [running_child]}])), '')
        ok_children = [{'pid': 8, 'disposition': 'terminated', 'exitStatus': 0},
                       {'pid': 9, 'disposition': 'quarantined', 'namespace': '/ldc/quarantine'},
                       {'pid': 10, 'disposition': 'process_gone'}]
        self.assertEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'descendants': ok_children}])), '')
        self.assertNotEqual(_evidence_error(evidence(
            oldProcesses=[{**base, 'descendants': [{'pid': 8, 'disposition': 'terminated'}]}])), '')


if __name__ == '__main__':
    unittest.main()