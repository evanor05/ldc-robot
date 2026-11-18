"""Bounded durable goal identity/evidence storage; no ROS or motion side effects."""
import json
from pathlib import Path
import re
import sqlite3
import time

ID = re.compile(r'^[0-9a-f]{32}$')
TERMINAL = {'rejected', 'succeeded', 'canceled', 'aborted'}
APPLICATION_ID = 0x4C444341  # LDCA


class ActionJournal:
    def __init__(self, filename, *, history_limit=128, pending_limit=256):
        self.filename = str(filename)
        self.history_limit = history_limit
        self.pending_limit = pending_limit
        self.connection = None
        self.closed = False
        self.error = ''

    def _db(self):
        if self.closed:
            raise RuntimeError('目标账本已经关闭')
        if self.connection is None:
            existing = False
            if self.filename != ':memory:':
                path = Path(self.filename)
                if path.is_symlink() or path.parent.is_symlink():
                    raise ValueError('目标账本不允许符号链接')
                path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                existing = path.exists() and path.stat().st_size > 0
            # Fail immediately on contention; never wait for a DB lock on the
            # ROS executor. Local fsync duration is not a hard realtime bound.
            db = sqlite3.connect(self.filename, timeout=0)
            try:
                db.execute('PRAGMA synchronous=FULL')
                application = db.execute('PRAGMA application_id').fetchone()[0]
                version = db.execute('PRAGMA user_version').fetchone()[0]
                if (existing and application != APPLICATION_ID) or version not in (0, 1):
                    raise ValueError('不是受支持的 LDC 目标账本，拒绝修改')
                db.execute(f'PRAGMA application_id={APPLICATION_ID}')
                db.execute('PRAGMA user_version=1')
                db.execute('''CREATE TABLE IF NOT EXISTS goals(
                    uuid TEXT PRIMARY KEY, prepared_at REAL NOT NULL,
                    metadata TEXT NOT NULL, outcome TEXT, terminal_at REAL)''')
                db.execute('''CREATE TABLE IF NOT EXISTS evidence(
                    id INTEGER PRIMARY KEY, uuid TEXT NOT NULL,
                    at REAL NOT NULL, kind TEXT NOT NULL, data TEXT NOT NULL)''')
                db.commit()
                if self.filename != ':memory:':
                    Path(self.filename).chmod(0o600)
            except Exception:
                db.close()
                raise
            self.connection = db
        return self.connection

    @staticmethod
    def _identifier(identifier):
        if not isinstance(identifier, str) or not ID.fullmatch(identifier) or identifier == '0' * 32:
            raise ValueError('无效目标 UUID')
        return identifier

    @staticmethod
    def _json(value, limit):
        text = json.dumps(value, ensure_ascii=False, allow_nan=False)
        if len(text.encode()) > limit:
            raise ValueError('目标账本记录超过容量限制')
        return text

    def prepare(self, identifier, metadata):
        identifier = self._identifier(identifier)
        if not isinstance(metadata, dict):
            raise ValueError('目标元数据必须为对象')
        data = self._json(metadata, 16384)
        db = self._db()
        with db:
            db.execute('BEGIN IMMEDIATE')
            if db.execute('SELECT count(*) FROM goals WHERE outcome IS NULL').fetchone()[0] >= self.pending_limit:
                raise ValueError('未确认目标账本已满，请先完成恢复或人工归档')
            db.execute('INSERT INTO goals VALUES(?,?,?,NULL,NULL)', (identifier, time.time(), data))

    def terminal(self, identifier, outcome):
        identifier = self._identifier(identifier)
        if outcome not in TERMINAL:
            raise ValueError('取消请求、ACK 或未知状态不能记为终态')
        db = self._db()
        with db:
            db.execute('BEGIN IMMEDIATE')
            row = db.execute('SELECT outcome FROM goals WHERE uuid=?', (identifier,)).fetchone()
            if row is None or row[0] not in (None, outcome):
                raise ValueError('目标记录缺失或终态冲突')
            db.execute('UPDATE goals SET outcome=?,terminal_at=? WHERE uuid=?',
                       (outcome, time.time(), identifier))
            db.execute('''DELETE FROM goals WHERE outcome IS NOT NULL AND uuid NOT IN
                (SELECT uuid FROM goals WHERE outcome IS NOT NULL ORDER BY terminal_at DESC LIMIT ?)''',
                       (self.history_limit,))
            db.execute('DELETE FROM evidence WHERE uuid NOT IN (SELECT uuid FROM goals)')

    def event(self, identifier, kind, data):
        identifier = self._identifier(identifier)
        text = self._json(data, 2048)
        db = self._db()
        with db:
            db.execute('BEGIN IMMEDIATE')
            if not db.execute('SELECT 1 FROM goals WHERE uuid=?', (identifier,)).fetchone():
                raise ValueError('目标记录缺失')
            db.execute('INSERT INTO evidence(uuid,at,kind,data) VALUES(?,?,?,?)',
                       (identifier, time.time(), str(kind)[:80], text))
            db.execute('''DELETE FROM evidence WHERE uuid=? AND id NOT IN
                (SELECT id FROM evidence WHERE uuid=? ORDER BY id DESC LIMIT 32)''',
                       (identifier, identifier))

    def pending(self):
        result = []
        for identifier, prepared_at, text in self._db().execute(
                'SELECT uuid,prepared_at,metadata FROM goals WHERE outcome IS NULL ORDER BY prepared_at'):
            metadata = json.loads(text)
            if not isinstance(metadata, dict):
                raise ValueError('目标元数据损坏')
            result.append({'goalUUID': self._identifier(identifier),
                           'preparedAt': prepared_at, 'metadata': metadata})
        return result

    def snapshot(self):
        if self.connection is None:
            return {'path': self.filename, 'loaded': False, 'pending': None, 'error': self.error}
        return {'path': self.filename, 'loaded': True,
                'pending': self.connection.execute('SELECT count(*) FROM goals WHERE outcome IS NULL').fetchone()[0],
                'error': self.error}

    def close(self):
        self.closed = True
        if self.connection is not None:
            self.connection.close()
            self.connection = None
