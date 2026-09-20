#!/usr/bin/env python3
"""Verify captured baseline against the live checkout or internal archive metadata."""
import argparse,hashlib,json,pathlib,subprocess,sys
parser=argparse.ArgumentParser()
parser.add_argument('--archive-only',action='store_true',help='check captured baseline metadata without comparing the current Git checkout')
a=parser.parse_args()
OUT=pathlib.Path(__file__).resolve().parents[1]
ROOT=OUT.parents[2]
manifest=json.loads((OUT/'baseline/source_manifest.json').read_text())
env=json.loads((OUT/'baseline/environment.json').read_text())
errors=[]
if a.archive_only:
    pairs=[('HEAD',env.get('head'),manifest.get('head')),
           ('branch',env.get('branch'),manifest.get('branch')),
           ('staged diff hash',env.get('staged_diff_sha256'),manifest.get('staged_diff_sha256')),
           ('unstaged diff hash',env.get('unstaged_diff_sha256'),manifest.get('unstaged_diff_sha256'))]
    for name,left,right in pairs:
        if left!=right: errors.append(f'captured {name} disagrees between environment and source manifest')
    status_file=OUT/'baseline/worktree_status.txt'
    if not status_file.is_file(): errors.append('missing baseline/worktree_status.txt')
    elif status_file.read_text().rstrip('\n')!=manifest.get('worktree_status_porcelain_v2'):
        errors.append('captured worktree status file disagrees with source manifest')
    count=env.get('source_snapshot',{}).get('file_count')
    if count!=len(manifest.get('files',[])): errors.append('captured snapshot file count disagrees with environment metadata')
    if errors:
        print('ARCHIVED BASELINE METADATA VERIFICATION FAILED:'); print('\n'.join(errors)); sys.exit(1)
    print(f'PASS: archived HEAD/branch/diff/status metadata is internally consistent; captured source files={count}')
    print('LIMIT: current HEAD, branch, worktree status and submodules were not compared to the captured checkout.')
    sys.exit(0)
def git(*args): return subprocess.check_output(['git',*args],cwd=ROOT,stderr=subprocess.DEVNULL).decode(errors='replace').rstrip('\n')
def sha(s): return hashlib.sha256(s.encode()).hexdigest()
if git('rev-parse','HEAD') != env['head']: errors.append('HEAD changed')
if git('branch','--show-current') != env['branch']: errors.append('branch changed or detached status changed')
status=git('status','--porcelain=v2','--branch')
if status != manifest['worktree_status_porcelain_v2']: errors.append('porcelain v2 status changed since capture')
unstaged=git('diff','--no-ext-diff','--binary')
staged=git('diff','--cached','--no-ext-diff','--binary')
if sha(unstaged)!=env['unstaged_diff_sha256']: errors.append('unstaged tracked diff hash changed')
if sha(staged)!=env['staged_diff_sha256']: errors.append('staged tracked diff hash changed')
if git('submodule','status').splitlines()!=[x for x in manifest['submodule_status']]: errors.append('submodule status changed')
if errors:
 print('BASELINE VERIFICATION FAILED:'); print('\n'.join(errors)); sys.exit(1)
print('PASS: HEAD, branch, porcelain v2 status, staged/unstaged diff hashes and submodule revisions match capture')
print('NOTE: files subsequently created inside the already-untracked audit output directory are expected audit artifacts.')
