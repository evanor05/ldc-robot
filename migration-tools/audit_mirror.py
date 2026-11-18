#!/usr/bin/env python3
"""Verify sanitized copied content; never emits source literals or their hashes."""
import hashlib
import json
from pathlib import Path
import re
import sys
sys.path.insert(0, str(Path(__file__).parent))
from prepare_mirror import ROOT, REVIEWED_LITERAL_LINES, SOURCES, KNOWN, URL_AUTH

manifest = json.loads((ROOT/'copy-report.json').read_text())
literals = set()
for rel, numbers in REVIEWED_LITERAL_LINES.items():
    prefix = next(k for k in SOURCES if rel.startswith(k+'/'))
    original = SOURCES[prefix]/rel[len(prefix)+1:]
    lines = original.read_text().splitlines()
    for n in numbers:
        match = re.search(r'["\']([^"\']+)["\']', lines[n-1])
        if match and len(match[1]) >= 4:
            literals.add(match[1])
findings = []
counts = {'text_files':0,'binary_assets':0,'known_literals_checked':len(literals)}
for e in manifest['copied']:
    p = ROOT/e['path']
    data = p.read_bytes()
    if e['text']:
        counts['text_files'] += 1
        text = data.decode('utf-8')
        # Check every copied text file against exact source credentials in memory.
        for value in literals:
            if value in text:
                findings.append({'path':e['path'],'kind':'known_sensitive_literal'})
        for kind, pattern in [('token_signature',KNOWN),('url_credentials',URL_AUTH),
                              ('private_key',re.compile(r'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----'))]:
            if pattern.search(text): findings.append({'path':e['path'],'kind':kind})
    else:
        counts['binary_assets'] += 1
    if hashlib.sha256(data).hexdigest() != e['sha256']:
        findings.append({'path':e['path'],'kind':'manifest_hash_mismatch'})
report = {'counts':counts,'findings':findings,
          'limits':'Heuristic scan is not proof of no secrets. Binary model/image assets not decoded. No auth stores opened. Exact known source literals only kept in process memory.'}
(ROOT/'sensitive-scan-report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))