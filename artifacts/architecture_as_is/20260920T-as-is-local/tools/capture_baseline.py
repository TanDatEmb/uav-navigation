#!/usr/bin/env python3
"""Capture the local behavior-audit source inputs without touching the checkout."""
from __future__ import annotations
import hashlib, json, os, pathlib, shutil, subprocess, sys
from datetime import datetime, timezone
ROOT = pathlib.Path(__file__).resolve().parents[4]
OUT = pathlib.Path(__file__).resolve().parents[1]
SNAP = OUT / 'baseline' / 'source_snapshot'
MANIFEST = OUT / 'baseline' / 'source_manifest.json'
EXTS = {'.cpp','.cc','.c','.h','.hh','.hpp','.hxx','.msg','.srv','.action','.yaml','.yml','.xml','.py','.cmake','.txt','.launch','.rviz','.urdf','.xacro','.sdf','.sh','.json','.md'}
SPECIAL = {'CMakeLists.txt','package.xml','Makefile','AGENTS.md','compile_commands.json'}
EXCLUDE_PARTS = {'.git','build','install','log','.artifacts','runtime_evidence','bags','rosbags'}
SECRET_NAMES = {'.env','.env.local','.env.production','id_rsa','id_ed25519'}

def sha(data: bytes) -> str: return hashlib.sha256(data).hexdigest()
def git(*args: str) -> str:
    return subprocess.check_output(['git', *args], cwd=ROOT, stderr=subprocess.DEVNULL).decode(errors='replace').rstrip('\n')
def eligible(p: pathlib.Path) -> bool:
    rel = p.relative_to(ROOT)
    if any(part in EXCLUDE_PARTS for part in rel.parts): return False
    if p.name.lower() in SECRET_NAMES or any(x in p.name.lower() for x in ('credential','secret','token')): return False
    if rel.parts[:2] == ('src','external'): return False  # submodules are provenance-recorded separately
    if any(part in {'3rdparty','vendor'} for part in rel.parts): return False
    if rel.parts[0] not in {'src','config','docs','tools'} and len(rel.parts) > 1: return False
    if rel.parts[0] == 'docs' and not (rel.parts[:2] == ('docs','safety') and p.name in {'runtime_safety_current.md','runtime_safety_index.md'}): return False
    if rel.parts[0] == 'tools' and rel.as_posix() != 'tools/validate_runtime_safety_ledger.py': return False
    if rel.parts[0] == 'src' and p.suffix.lower() not in EXTS and p.name not in SPECIAL: return False
    if rel.parts[0] == 'config' and p.suffix.lower() not in EXTS: return False
    return p.name in SPECIAL or p.suffix.lower() in EXTS

def run() -> int:
    before = git('status','--porcelain=v2','--branch')
    paths = sorted((p for p in ROOT.rglob('*') if p.is_file() and eligible(p)), key=lambda p:p.relative_to(ROOT).as_posix())
    entries=[]
    for p in paths:
        rel=p.relative_to(ROOT).as_posix(); data=p.read_bytes(); dest=SNAP/rel; dest.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(p,dest)
        try: lines=data.decode('utf-8').count('\n') + (1 if data and not data.endswith(b'\n') else 0)
        except UnicodeDecodeError: lines=None
        entries.append({'path':rel,'sha256':sha(data),'bytes':len(data),'lines':lines,'source_kind':'working_tree'})
    # The active compilation database is preserved as a diagnostic input even though
    # this does not assert that it matches the current source tree.
    cc=ROOT/'build/compile_commands.json'
    if cc.is_file() and not any(e['path']=='build/compile_commands.json' for e in entries):
        data=cc.read_bytes(); dest=SNAP/'build/compile_commands.json'; dest.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(cc,dest)
        entries.append({'path':'build/compile_commands.json','sha256':sha(data),'bytes':len(data),'lines':data.count(b'\n'),'source_kind':'generated_build_metadata_unverified'})
    submods=[]
    try:
        for row in git('submodule','status').splitlines():
            submods.append(row)
    except Exception as e: submods=[f'ERROR: {e}']
    remotes=[]
    try:
        for row in git('config','--get-regexp',r'^remote\..*\.url').splitlines():
            key, val=row.split(' ',1)
            import re
            val=re.sub(r'(https?://)[^/@]+(:[^/@]+)?@',r'\1[REDACTED]@',val)
            val=re.sub(r'(://)[^/@]+(:[^/@]+)?@',r'\1[REDACTED]@',val)
            remotes.append({'name':key,'url':val})
    except Exception: pass
    try: unstaged=git('diff','--no-ext-diff','--binary')
    except Exception: unstaged=''
    try: staged=git('diff','--cached','--no-ext-diff','--binary')
    except Exception: staged=''
    try: status_json=git('status','--porcelain=v2','--branch')
    except Exception: status_json=before
    meta={
      'captured_utc':datetime.now(timezone.utc).isoformat(),
      'repository_root':str(ROOT),'head':git('rev-parse','HEAD'),'branch':git('branch','--show-current') or None,
      'detached':not bool(git('branch','--show-current')),'worktree_status_porcelain_v2':status_json,
      'unstaged_diff_sha256':sha(unstaged.encode()),'staged_diff_sha256':sha(staged.encode()),
      'unstaged_diff_bytes':len(unstaged.encode()),'staged_diff_bytes':len(staged.encode()),
      'remotes_redacted':remotes,'submodule_status':submods,'source_files':len(entries),
      'snapshot_scope':'first-party source/config/build/interface text under src and config, current safety contract/index, root agent contract and safety-ledger validator; excludes artifacts, runtime evidence, build products and external submodule payloads',
      'files':entries}
    MANIFEST.parent.mkdir(parents=True,exist_ok=True); MANIFEST.write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
    (OUT/'baseline'/'worktree_status.txt').write_text(status_json+'\n')
    (OUT/'baseline'/'source_list.txt').write_text('\n'.join(e['path'] for e in entries)+'\n')
    # Recompute immediately to catch races during capture.
    drift=[]
    for e in entries:
        p=ROOT/e['path']
        if not p.is_file() or sha(p.read_bytes()) != e['sha256']: drift.append(e['path'])
    after=git('status','--porcelain=v2','--branch')
    if after != before: drift.append('git status changed during capture')
    meta['capture_drift']=drift
    MANIFEST.write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'files':len(entries),'drift':drift,'head':meta['head'],'branch':meta['branch'],'snapshot':str(SNAP)},ensure_ascii=False))
    return 1 if drift else 0

if __name__=='__main__': sys.exit(run())
