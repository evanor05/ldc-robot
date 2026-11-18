#!/usr/bin/env python3
"""Parse only; never imports copied code or executes copied shell scripts."""
import ast
import collections
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
report = json.loads((ROOT/'copy-report.json').read_text())
counts = collections.Counter()
issues = []
for entry in report['copied']:
    p = ROOT/entry['path']
    kind = None
    try:
        if p.suffix == '.py':
            kind = 'python'
            ast.parse(p.read_bytes(), filename=entry['path'])
        elif p.suffix == '.json':
            kind = 'json'
            json.loads(p.read_bytes())
        elif p.suffix in {'.xml', '.urdf', '.xacro', '.svg', '.qrc', '.ui'}:
            kind = 'xml'
            ET.parse(p)
        elif p.suffix in {'.sh', '.bash'}:
            kind = 'shell'
            result = subprocess.run(['bash', '-n', str(p)], capture_output=True)
            if result.returncode:
                raise ValueError('bash parse failure (output suppressed)')
        if kind: counts[kind+'_passed'] += 1
    except Exception as exc:
        counts[str(kind)+'_failed'] += 1
        # Never report parser messages: they can echo file contents.
        issues.append({'path':entry['path'],'kind':kind,'error_type':type(exc).__name__,
                       'line':getattr(exc,'lineno',None)})
result = {'counts':dict(counts),'issues':issues,
          'limits':'Syntax only; no ROS/ESP-IDF/Node builds, no dependency installation. JSONC and intentionally malformed fixtures can fail strict parsers.'}
(ROOT/'validation-report.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))