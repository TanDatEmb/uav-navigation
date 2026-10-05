#!/usr/bin/env python3
"""Verify local preservation and canonical mapping before repository replacement."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile


def digest(path):
    with path.open('rb') as stream:
        result = hashlib.file_digest(stream, 'sha256').hexdigest()
    return result


def check_publish_index(root):
    paths = subprocess.check_output(['git', '-C', str(root), 'ls-files', '-z', '--', 'docs']).decode().split('\0')
    for name in filter(None, paths):
        if name.startswith(('docs/refactor/', 'docs/adr/')) or not (root / name).is_file():
            raise ValueError('missing or retired document remains in publish index: ' + name)
        if name.startswith('docs/architecture/') and name.endswith('.md') and name != 'docs/architecture/SYSTEM_DESIGN.md':
            raise ValueError('competing design remains in publish index: ' + name)


def verify(root, backup):
    check_publish_index(root)
    for line in (backup / 'SHA256SUMS').read_text().splitlines():
        expected, name = line.split(None, 1)
        path = (backup / name).resolve()
        if not path.is_relative_to(backup.resolve()) or digest(path) != expected:
            raise ValueError(f'backup checksum mismatch: {name}')
    restored = backup / 'restore-check.git'
    subprocess.run(['git', '--git-dir=' + str(restored), 'fsck', '--full'], check=True, capture_output=True)
    refs = subprocess.check_output(['git', '--git-dir=' + str(restored), 'show-ref'], text=True)
    if set(refs.splitlines()) != set((backup / 'refs.txt').read_text().splitlines()):
        raise ValueError('restored refs differ from preservation snapshot')
    report = json.loads((backup / 'restore-verify.json').read_text())
    if report.get('bundle_fsck') != 'PASS' or report.get('wip_files_verified', 0) <= 0:
        raise ValueError('missing completed WIP recovery verification')
    for item in json.loads((backup / 'github-manifest.json').read_text()):
        if digest(backup / item['path']) != item['sha256']:
            raise ValueError('GitHub metadata checksum mismatch: ' + item['path'])
    rows = list(csv.DictReader((root / 'docs/evidence/documentation_manifest.csv').open()))
    if not rows or len({r['source'] for r in rows}) != len(rows):
        raise ValueError('empty/duplicate retirement manifest')
    for row in rows:
        if (root / row['source']).exists():
            raise ValueError('retired path remains live: ' + row['source'])
        if not row['destination'].strip():
            raise ValueError('missing content mapping: ' + row['source'])
        path = backup / 'retired-working-docs' / row['source']
        if path.is_file():
            if digest(path) != row['sha256']:
                raise ValueError('retired WIP checksum mismatch: ' + row['source'])
            continue
        found = False
        for name in ['retired-plans.tar.gz', 'retirement-snapshot.tar.gz', 'source-wip.tar.gz']:
            with tarfile.open(backup / name) as archive:
                members = [m for m in archive.getmembers() if m.isfile() and m.name.removeprefix('./') == row['source']]
                if members:
                    found = hashlib.sha256(archive.extractfile(members[0]).read()).hexdigest() == row['sha256']
                    break
        if not found:
            raise ValueError('retired content missing/mismatched: ' + row['source'])
    dispositions = list(csv.DictReader((root / 'docs/evidence/commit_disposition.csv').open()))
    if not dispositions or any(r['status'] not in {'KEEP', 'DUPLICATE', 'DEFER', 'REJECT'} or not r['reason'].strip() for r in dispositions):
        raise ValueError('commit disposition incomplete')
    subprocess.run(['python3', str(root / 'tools/check_documentation.py'), str(root), 'docs'], check=True)
    print(f'preservation: {len(refs.splitlines())} refs; {report["wip_files_verified"]} recovered WIP files; {len(rows)} retired documents')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backup', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        verify(args.root.resolve(), args.backup.resolve())
    except (OSError, ValueError, subprocess.CalledProcessError, tarfile.TarError) as error:
        print(f'BACKUP_MIGRATION_RESULT=FAIL: {error}')
        return 1
    print('BACKUP_MIGRATION_RESULT=PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
