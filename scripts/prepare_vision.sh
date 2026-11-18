#!/usr/bin/env bash
set -eo pipefail
VISION_MODEL_DIR="${LDC_MODEL_DIR:-$HOME/.local/share/ldc/models}"
python3 - "$VISION_MODEL_DIR" <<'PY'
from pathlib import Path
import hashlib,sys
source=Path('/home/<user>/ros-car-migration-audit/yolo-integration/yolo26n.pt')
target=Path(sys.argv[1])/'yolo26n.pt'
target.parent.mkdir(parents=True,exist_ok=True)
if not target.exists():
    with target.open('xb') as out,source.open('rb') as src:out.write(src.read())
assert hashlib.sha256(target.read_bytes()).digest()==hashlib.sha256(source.read_bytes()).digest(),'现有模型不同，未覆盖'
(target.parent/'yolo26n.sha256').write_text(hashlib.sha256(target.read_bytes()).hexdigest()+'  yolo26n.pt\n')
print('模型已就绪:',target)
PY
