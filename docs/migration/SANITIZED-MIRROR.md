> 本文记录脱敏镜像的制作过程，属于迁移审计档案；入口说明见 [迁移过程说明](../迁移过程说明.md)。

# Sanitized source mirror

Prepared locally from read-only source trees. No Git commit, push, dependency
installation, upload, or authentication-store access was performed by the mirror
preparation scripts. Originals and encrypted backup are handled separately.

## Coverage

| Source under `/home/<user>/` | Copied files |
| --- | ---: |
| `xuegeros_ws/src` | 485 |
| `microros_ws/src` | 45 |
| `leap_demo` (both `leap_ros_mcu_driver` and `leap_cam`) | 8,517 |
| `Ros_Vue_x86` | 21 |
| `lichtblick` | 2,235 |
| **Total** | **11,303** |

Copied source bytes: **116,091,524** (before migration docs/reports).
Includes launch files, ROS interfaces/packages, configuration, robot meshes,
navigation maps, firmware components, frontend/backend, vendored component
source, licenses, lockfiles, tests, assets and documentation. Managed components
were intentionally retained to preserve locally available dependency source;
their generated build/install/cache output was excluded. Existing source
`.github` workflows are retained for coverage: review/disable upstream release,
deployment and publish workflows before enabling Actions in the destination.

`copy-report.json` records every copied file with size and SHA-256 of its
**sanitized destination bytes**, redaction records, all source roots, and 85
excluded entries. Excluded directories are recorded once at subtree granularity,
not as an expensive recursive inventory of their contents. No original secret
values or secret-value hashes are placed in reports.

## Exclusions

Git metadata/history, node_modules, generated build/install/dist/log directories,
Python/Yarn/tool caches, backups, runtime logs/PIDs, compiled objects/libraries/
firmware binaries, runtime recordings, and sdkconfig.old are excluded.
`microros_ws/src/micro.zip` is a redundant archive containing Git metadata;
expanded micro-ROS source is included instead. Symlinks are never followed;
credential containers (`.env` except templates, `.npmrc`, key/certificate stores,
etc.) are excluded without opening if encountered. See the script for exact
rules and the manifest for actual exclusions. Maps (`.pgm`/YAML), mesh/model,
image, PDF, font and intentional extension test fixture assets are retained.
The 114 non-UTF8/binary assets are **not decoded for a semantic secret scan**.

## Sanitization and local configuration

35 files have sanitized content (40 redaction records; a propagated record may
cover multiple occurrences). Reviewed hardcoded sudo password, Wi-Fi SSIDs and
Wi-Fi passwords were replaced with `REDACTED_SECRET` / `REDACTED_WIFI_SSID`.
Their repeated occurrences in docs, source defaults, configs, user paths and
network names were also replaced everywhere in copied text. This intentionally
may leave local paths or network defaults unusable; reconfigure for the target
machine. Never commit working credentials.

The placeholders are **literal templates, not automatic environment lookups**.
Before use, provide Wi-Fi through the firmware provisioning UI/NVS or locally
configured ESP-IDF settings; update `set_wifi.py` locally if needed. For the ROS
launch sudo password, prefer removing password piping and configuring narrowly
scoped privilege handling; do not run with a placeholder or restore a password
in tracked source. Keep deployment-specific credentials outside this mirror.

The sanitizer first identifies reviewed literal assignments, then propagates
those exact values in memory to all copied text. Generic token-signature and
URL-userinfo scans also run. Non-secret parser tokens, pin `PWDN` identifiers,
NVS key names, UI labels and environment/GitHub-secret expressions are retained
unchanged rather than broadly damaging code with false-positive redactions.
`sensitive-scan-report.json` reports **zero residual known-credential literals,
recognized token signatures, private-key headers, URL credentials, or manifest
hash mismatches**. This heuristic result is not a guarantee that all possible
secrets have been found. Private IPs, hardware identifiers, author emails and
upstream URLs can remain; review those according to your privacy policy.

## Validation and reproducibility

Scripts in `migration-tools/` only modify this repository. Run with Python 3:

```sh
python3 migration-tools/prepare_mirror.py
python3 migration-tools/validate_mirror.py
python3 migration-tools/audit_mirror.py
```

The builder overwrites selected mirrored source files; it does not automatically
remove stale files after rule/source changes. Start with a clean destination
source subtree when changing scope. Do not run copied application scripts for
validation: they may connect to hardware or networks.

Syntax passes: **372 Python, 88 strict JSON, 178 XML-family, 27 shell**.
13 strict-parser failures are itemized in `validation-report.json`: six legacy
vendored GoogleTest Python scripts (Python 2 syntax), five Lichtblick JSONC
configuration files, and two micro-ROS XML resource templates/fragments.
These unredacted files were retained rather than rewritten. No ROS/ESP-IDF/C++/
Node build or integration test was performed, and no build success is claimed.
Exact originals belong only in the parent's separate backup, not this mirror.