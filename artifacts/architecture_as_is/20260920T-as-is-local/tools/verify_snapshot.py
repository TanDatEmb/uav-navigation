#!/usr/bin/env python3
"""Verify captured source bytes against the live checkout or archive alone."""
import argparse,hashlib,json,pathlib,sys
parser=argparse.ArgumentParser()
parser.add_argument('--archive-only',action='store_true',help='check snapshot hashes against the manifest without comparing a live checkout')
a=parser.parse_args()
OUT=pathlib.Path(__file__).resolve().parents[1]
ROOT=OUT.parents[2]
manifest=json.loads((OUT/'baseline/source_manifest.json').read_text())
errors=[]
for item in manifest['files']:
    rel=item['path']; expected=item['sha256']
    snap=OUT/'baseline/source_snapshot'/rel
    targets=[('snapshot',snap)]
    if not a.archive_only: targets.append(('live',ROOT/rel))
    for label,path in targets:
        if not path.is_file(): errors.append(f'MISSING {label}: {rel}'); continue
        got=hashlib.sha256(path.read_bytes()).hexdigest()
        if got!=expected: errors.append(f'DRIFT {label}: {rel} expected={expected} got={got}')
if errors:
    print('SOURCE SNAPSHOT VERIFICATION FAILED')
    print('\n'.join(errors)); sys.exit(1)
if a.archive_only:
    print(f'PASS: {len(manifest["files"])} captured files match exact SHA-256 in archived source snapshot')
    print('LIMIT: live checkout comparison was explicitly skipped; this verifies the archive, not current source equivalence.')
else:
    print(f'PASS: {len(manifest["files"])} captured files match exact SHA-256 in snapshot and live checkout')
print('NOTE: generated report files and external submodule payloads are outside captured source hash scope.')
