from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest
import uuid

from scripts.action_journal import ActionJournal


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'actions.sqlite3'

    def journal(self, **kwargs):
        journal = ActionJournal(self.path, **kwargs)
        self.addCleanup(journal.close)
        return journal

    def test_prepared_identity_and_metadata_survive_owner_restart(self):
        journal = self.journal()
        self.assertIsNone(journal.snapshot()['pending'])
        identifier = uuid.uuid4().hex
        metadata = {'bootId': 'old-boot', 'runEpoch': 'old-run', 'actionName': '/old/navigate_to_pose'}
        journal.prepare(identifier, metadata)
        journal.close()
        reopened = self.journal()
        self.assertEqual(reopened.pending()[0]['goalUUID'], identifier)
        self.assertEqual(reopened.pending()[0]['metadata'], metadata)
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)

    def test_committed_prepare_survives_exit_without_cleanup(self):
        identifier = uuid.uuid4().hex
        source = '''import os,sys
from scripts.action_journal import ActionJournal
journal=ActionJournal(sys.argv[1])
journal.prepare(sys.argv[2],{'source':'abrupt-owner'})
os._exit(33)
'''
        result = subprocess.run([sys.executable, '-c', source, str(self.path), identifier],
                                cwd=Path(__file__).resolve().parents[2], timeout=5)
        self.assertEqual(result.returncode, 33)
        self.assertEqual(self.journal().pending()[0]['goalUUID'], identifier)

    def test_ack_unknown_and_conflicting_terminal_cannot_erase_pending(self):
        journal = self.journal()
        identifier = uuid.uuid4().hex
        journal.prepare(identifier, {})
        for outcome in ['canceling', 'ACK', 'unknown']:
            with self.assertRaises(ValueError):
                journal.terminal(identifier, outcome)
        self.assertEqual(len(journal.pending()), 1)
        journal.terminal(identifier, 'canceled')
        journal.terminal(identifier, 'canceled')
        with self.assertRaises(ValueError):
            journal.terminal(identifier, 'succeeded')
        self.assertEqual(journal.pending(), [])

    def test_duplicate_id_and_invalid_metadata_are_not_overwritten(self):
        journal = self.journal()
        identifier = uuid.uuid4().hex
        journal.prepare(identifier, {'source': 'original'})
        with self.assertRaises(sqlite3.IntegrityError):
            journal.prepare(identifier, {'source': 'replacement'})
        self.assertEqual(journal.pending()[0]['metadata']['source'], 'original')
        for data in [{'x': float('nan')}, {'payload': 'x' * 20000}]:
            with self.assertRaises(ValueError):
                journal.prepare(uuid.uuid4().hex, data)

    def test_terminal_history_is_bounded_but_unconfirmed_goals_remain(self):
        journal = self.journal(history_limit=2, pending_limit=2)
        retained = uuid.uuid4().hex
        journal.prepare(retained, {'must': 'remain'})
        for _ in range(5):
            identifier = uuid.uuid4().hex
            journal.prepare(identifier, {})
            journal.terminal(identifier, 'succeeded')
        self.assertEqual([g['goalUUID'] for g in journal.pending()], [retained])
        self.assertEqual(journal.connection.execute('SELECT count(*) FROM goals').fetchone()[0], 3)
        journal.prepare(uuid.uuid4().hex, {})
        with self.assertRaises(ValueError):
            journal.prepare(uuid.uuid4().hex, {})

    def test_evidence_is_bounded_and_lock_failure_does_not_wait_or_modify(self):
        journal = self.journal()
        identifier = uuid.uuid4().hex
        journal.prepare(identifier, {})
        for i in range(40):
            journal.event(identifier, 'cancel_reply', {'sequence': i})
        self.assertEqual(journal.connection.execute('SELECT count(*) FROM evidence').fetchone()[0], 32)
        self.assertEqual(journal.connection.execute('PRAGMA busy_timeout').fetchone()[0], 0)
        other = sqlite3.connect(self.path)
        self.addCleanup(other.close)
        other.execute('BEGIN EXCLUSIVE')
        with self.assertRaises(sqlite3.OperationalError):
            journal.prepare(uuid.uuid4().hex, {})
        other.rollback()
        self.assertEqual(len(journal.pending()), 1)

    def test_symlink_and_corrupt_store_fail_instead_of_becoming_empty(self):
        target = Path(self.temp.name) / 'not-a-journal'
        target.write_text('original evidence')
        self.path.symlink_to(target)
        with self.assertRaises(ValueError):
            self.journal().pending()
        self.assertEqual(target.read_text(), 'original evidence')
        corrupt = ActionJournal(target)
        self.addCleanup(corrupt.close)
        with self.assertRaises(sqlite3.DatabaseError):
            corrupt.pending()
        self.assertEqual(target.read_text(), 'original evidence')

    def test_an_unrelated_sqlite_database_is_not_modified(self):
        db = sqlite3.connect(self.path)
        db.execute('CREATE TABLE unrelated(value TEXT)')
        db.execute('INSERT INTO unrelated VALUES("keep")')
        db.commit()
        db.close()
        original = self.path.read_bytes()
        with self.assertRaises(ValueError):
            self.journal().pending()
        self.assertEqual(self.path.read_bytes(), original)
