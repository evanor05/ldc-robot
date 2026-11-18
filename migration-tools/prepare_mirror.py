#!/usr/bin/env python3
"""Local-only, read-only-source mirror builder. Never invokes Git or networking."""
import ast
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'xuegeros_ws/src': Path('/home/<user>/xuegeros_ws/src'),
    'microros_ws/src': Path('/home/<user>/microros_ws/src'),
    'leap_demo': Path('/home/<user>/leap_demo'),
    'Ros_Vue_x86': Path('/home/<user>/Ros_Vue_x86'),
    'lichtblick': Path('/home/<user>/lichtblick'),
}
DIRS = {'.git', 'node_modules', 'build', 'install', 'log', 'logs', 'dist',
        '__pycache__', '.cache', '.pytest_cache', '.mypy_cache', '.ruff_cache',
        '.pio', '.next', '.nuxt', 'coverage', '.turbo', '.venv', 'venv',
        'CMakeFiles', '.bugfix_backup', '.deploy_backup', 'git_backup'}
SUFFIXES = {'.pyc', '.pyo', '.o', '.obj', '.a', '.so', '.dll', '.exe', '.elf',
            '.bin', '.map', '.log', '.pid', '.bag', '.mcap', '.db3', '.tsbuildinfo'}
AUTH_NAMES = {'.npmrc', '.netrc', '.pypirc', '.git-credentials', '.dockercfg',
              'id_rsa', 'id_ed25519', 'credentials.json', 'service-account.json'}
KEY = r'(?:[A-Za-z0-9_.-]{0,80}(?:password|passwd|passphrase|pwd|ssid|psk|secret|token|api[_-]?key|access[_-]?key|private[_-]?key)[A-Za-z0-9_.-]{0,80})'
QUOTED = re.compile(r'(?P<head>[\"\']?'+KEY+r'[\"\']?\s*(?:[:=]|\s)\s*(?:\([A-Za-z_][\w *]*\)\s*)?)(?P<q>[\"\'])(?P<value>[^\n]*?)(?P=q)', re.I)
BARE = re.compile(r'^(?P<head>\s*(?:export\s+|#define\s+)?[\"\']?'+KEY+r'[\"\']?\s*(?:[:=]|\s)\s*)(?P<value>[^\s\"\'{}\[\](),;#]+)(?P<tail>\s*(?:[#;].*)?)$', re.I|re.M)
KNOWN = re.compile(r'\b(?:gh[pousr]_[A-Za-z0-9_]{20,}|github_pat_[A-Za-z0-9_]{20,}|AKIA[0-9A-Z]{16}|sk-[A-Za-z0-9_-]{20,}|xox[baprs]-[A-Za-z0-9-]{10,})\b')
URL_AUTH = re.compile(r'(https?://)[^\s/@:]+:[^\s/@]+@')
PLACEHOLDER = re.compile(r'^(?:|true|false|null|none|undefined|0|1|\d+|REDACTED_[A-Z_]+|\$.*|<.*>|\*+)$', re.I)

def sensitive_path(path):
    n = path.name.lower()
    return (n in AUTH_NAMES or (n.startswith('.env') and not any(x in n for x in ('example','template','sample')))
            or path.suffix.lower() in {'.pem', '.key', '.p12', '.pfx', '.jks', '.keystore'}
            or n in {'secrets.yaml','secrets.yml','secrets.json'})

def sanitize_line(text):
    changes = []
    def quoted(m):
        value = m['value']
        if PLACEHOLDER.match(value):
            return m[0]
        key = m['head'].upper()
        kind = 'WIFI_SSID' if 'SSID' in key else 'SECRET'
        changes.append({'line': text.count('\n', 0, m.start())+1, 'kind': kind})
        return m['head'] + m['q'] + 'REDACTED_'+kind + m['q']
    text = QUOTED.sub(quoted, text)
    def bare(m):
        v = m['value']
        # Preserve expressions/identifiers; sdkconfig / YAML / shell literal values are scrubbed.
        if PLACEHOLDER.match(v) or v.startswith(('self.', 'this.', 'process.', 'os.', 'config.', '&')):
            return m[0]
        if re.fullmatch(r'[A-Za-z_]\w*', v) and not (m['head'].lstrip().startswith('#define') or ':' in m['head'] or '=' in m['head']):
            return m[0]
        changes.append({'line': text.count('\n',0,m.start())+1, 'kind':'BARE_SECRET'})
        return m['head']+'REDACTED_SECRET'+m['tail']
    text = BARE.sub(bare, text)
    def known(m):
        changes.append({'line': text.count('\n',0,m.start())+1,'kind':'TOKEN_PATTERN'})
        return 'REDACTED_SECRET'
    text = KNOWN.sub(known,text)
    def url(m):
        changes.append({'line': text.count('\n',0,m.start())+1,'kind':'URL_CREDENTIAL'})
        return m[1]+'REDACTED_USER:REDACTED_SECRET@'
    text = URL_AUTH.sub(url,text)
    return text, changes

REVIEWED_LITERAL_LINES = {
    'xuegeros_ws/src/xuegeros_demo/launch/bringup_all.launch.py': {9},
    'leap_demo/leap_ros_mcu_driver/sdkconfig': {616, 617},
    'leap_demo/leap_ros_mcu_driver/main/network/wifi_app.cpp': {26, 27},
    'leap_demo/leap_cam/set_wifi.py': {19, 20},
}

def sanitize(text, path):
    output, changes = [], []
    trigger = re.compile(r'password|passwd|passphrase|pwd|ssid|psk|secret|token|api.?key|access.?key|private.?key|https?://[^ /]+@|gh[pousr]_|github_pat_|AKIA|sk-|xox[baprs]-', re.I)
    for number, line in enumerate(text.splitlines(keepends=True), 1):
        if number in REVIEWED_LITERAL_LINES.get(path, set()):
            line, findings = sanitize_line(line)
            changes.extend(dict(f, line=number) for f in findings)
        else:
            # Only universal credential signatures outside reviewed literal assignments.
            def scrub(m):
                changes.append({'line':number,'kind':'TOKEN_PATTERN'})
                return 'REDACTED_SECRET'
            line = KNOWN.sub(scrub, line)
            if URL_AUTH.search(line):
                changes.append({'line':number,'kind':'URL_CREDENTIAL'})
                line = URL_AUTH.sub(lambda m: m[1]+'REDACTED_USER:REDACTED_SECRET@',line)
        output.append(line)
    return ''.join(output), changes

def main():
    # Propagate reviewed credentials to every copied text occurrence (including docs/defaults).
    sensitive_literals = {}
    for rel, numbers in REVIEWED_LITERAL_LINES.items():
        prefix = next(k for k in SOURCES if rel.startswith(k+'/'))
        lines = (SOURCES[prefix]/rel[len(prefix)+1:]).read_text().splitlines()
        for n in numbers:
            match = re.search(r'["\']([^"\']+)["\']', lines[n-1])
            if match:
                sensitive_literals[match[1]] = 'REDACTED_WIFI_SSID' if 'ssid' in lines[n-1].lower() else 'REDACTED_SECRET'
    manifest = {'sources': {k:str(v) for k,v in SOURCES.items()}, 'copied': [], 'excluded': [], 'redactions': [], 'validation': [], 'scan': {}}
    for prefix, source in SOURCES.items():
        if not source.is_dir():
            raise RuntimeError('Missing source root: '+str(source))
        for base, dirs, files in os.walk(source, followlinks=False):
            base = Path(base)
            relbase = Path(prefix)/base.relative_to(source)
            for name in sorted(dirs[:]):
                src = base/name
                reason = None
                if src.is_symlink(): reason = 'symlink directory (not followed)'
                elif name == '.git': reason = 'Git metadata/history (not read)'
                elif name in DIRS: reason = 'generated/dependency/cache/backup directory'
                elif name in {'.ssh', '.aws', '.gnupg'}: reason = 'authentication directory (not read)'
                elif name in {'cache', 'unplugged', 'install-state'} and '.yarn' in src.parts: reason = 'Yarn dependency cache'
                if reason:
                    dirs.remove(name)
                    manifest['excluded'].append({'path':str(relbase/name),'reason':reason,'scope':'subtree'})
            for name in sorted(files):
                src = base/name
                rel = relbase/name
                reason = None
                if src.is_symlink(): reason = 'symlink (not followed; prevents external source traversal)'
                elif str(rel) == 'microros_ws/src/micro.zip': reason = 'duplicate source archive containing Git metadata; expanded source included instead'
                elif sensitive_path(src): reason = 'authentication/secret container (not opened)'
                elif name == '.git': reason = 'Git metadata pointer (not read)'
                elif src.suffix.lower() in SUFFIXES or name in {'CMakeCache.txt','compile_commands.json','.DS_Store','.yarn-integrity','install-state.gz','sdkconfig.old'}: reason = 'generated artifact/runtime data'
                if reason:
                    manifest['excluded'].append({'path':str(rel),'reason':reason,'scope':'file'})
                    continue
                data = src.read_bytes()
                is_text = b'\0' not in data
                if is_text:
                    try: text = data.decode('utf-8')
                    except UnicodeDecodeError: is_text = False
                changes = []
                if is_text:
                    text, changes = sanitize(text, str(rel))
                    for value, placeholder in sensitive_literals.items():
                        if value in text:
                            changes.append({'kind':'KNOWN_LITERAL_PROPAGATED','occurrences':text.count(value)})
                            text = text.replace(value, placeholder)
                    data = text.encode('utf-8')
                    manifest['scan']['text_files'] = manifest['scan'].get('text_files',0)+1
                else:
                    manifest['scan']['binary_files_not_content_scanned'] = manifest['scan'].get('binary_files_not_content_scanned',0)+1
                dest = ROOT/rel
                dest.parent.mkdir(parents=True,exist_ok=True)
                dest.write_bytes(data)
                dest.chmod(stat.S_IMODE(src.stat().st_mode) & 0o777)
                manifest['copied'].append({'path':str(rel),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'text':is_text,'sanitized':bool(changes)})
                if changes: manifest['redactions'].append({'path':str(rel),'changes':changes})
    (ROOT/'copy-report.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'copied_files':len(manifest['copied']),'excluded_entries':len(manifest['excluded']),'redacted_files':len(manifest['redactions']),'scan':manifest['scan']},indent=2))

if __name__ == '__main__': main()