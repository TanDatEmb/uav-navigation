#!/usr/bin/env python3
"""Verify file:line citations in design docs resolve against the working tree."""
import re,subprocess,sys,glob,os
REPO=sys.argv[1]
files=subprocess.run(['git','-C',REPO,'ls-files','-co','--exclude-standard'],capture_output=True,text=True).stdout.split()
files=[f for f in files if os.path.isfile(os.path.join(REPO,f))]
if not files: sys.exit('no files found: is REPO a git working tree?')
by_base={}
for f in files: by_base.setdefault(os.path.basename(f),[]).append(f)
pat=re.compile(r'`?([A-Za-z0-9_./-]+\.(?:hpp|cpp|py|yaml|msg|md))`?:(\d+)(?:-(\d+))?((?:,\d+(?:-\d+)?)*)')
bad=0;n=0;amb=set()
for doc in sorted(glob.glob(sys.argv[2]+'/**/*.md',recursive=True)):
  for ln,line in enumerate(open(doc),1):
    for m in pat.finditer(line):
      name=m.group(1); cands=[f for f in files if f.endswith('/'+name) or f==name] or by_base.get(os.path.basename(name),[])
      if not cands: continue   # WP artifact or non-repo file
      nums=[int(m.group(2))]+([int(m.group(3))] if m.group(3) else [])
      for extra in filter(None,m.group(4).split(',')): nums+= [int(x) for x in extra.split('-')]
      ok=False
      for c in cands:
        L=open(os.path.join(REPO,c),errors='replace').read().count('\n')
        if max(nums)<=L+1: ok=True
      if len(cands)>1: amb.add(name)
      n+=1
      if not ok: bad+=1; print(f'OUT_OF_RANGE {os.path.basename(doc)}:{ln} {m.group(0)} cands={cands}')
print(f'checked={n} out_of_range={bad} ambiguous_basenames={sorted(amb)}')
sys.exit(1 if bad else 0)
