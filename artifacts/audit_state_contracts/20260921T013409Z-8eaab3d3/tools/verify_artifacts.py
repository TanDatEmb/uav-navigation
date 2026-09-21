#!/usr/bin/env python3
"""Read-only structural, provenance and link checks; semantic completeness stays separate."""
from __future__ import annotations
import argparse, gzip, hashlib, html.parser, json, re, subprocess, sys
from pathlib import Path

OUT_REL=Path('artifacts/audit_state_contracts/20260921T013409Z-8eaab3d3')
def cmd(repo: Path,*args: str, text=True): return subprocess.check_output(['git',*args],cwd=repo,text=text,stderr=subprocess.PIPE)
def fail(errors:list[str],msg:str): errors.append(msg)
class Links(html.parser.HTMLParser):
    def __init__(self): super().__init__(); self.values=[]
    def handle_starttag(self,tag,attrs):
        for k,v in attrs:
            if k in ('href','src') and v: self.values.append(v)
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--repo',type=Path,required=True); ap.add_argument('--target-sha',required=True); a=ap.parse_args()
    repo=a.repo.resolve(); out=repo/OUT_REL; errors=[]; warnings=[]
    try: head=cmd(repo,'rev-parse','HEAD').strip()
    except Exception as e: print(json.dumps({'artifact_integrity':'FAIL','error':str(e)})); return 1
    model=json.loads((out/'model/architecture.json').read_text()); refs=[json.loads(x) for x in (out/'evidence/refs.jsonl').read_text().splitlines() if x]
    if model.get('target_sha')!=a.target_sha: fail(errors,'model TARGET_SHA mismatch')
    try: cmd(repo,'cat-file','-e',a.target_sha+'^{commit}')
    except subprocess.CalledProcessError: fail(errors,'TARGET_SHA object missing')
    ids=[]
    for key in ('objects','guards','transitions','contracts','scenarios'):
        ids.extend(x['id'] for x in model.get(key,[]))
    for obj in model.get('objects',[]): ids.extend(x['id'] for x in obj.get('fields',[]))
    if len(ids)!=len(set(ids)): fail(errors,'model IDs are not unique')
    ref_ids=[x.get('id') for x in refs]
    if len(ref_ids)!=len(set(ref_ids)): fail(errors,'evidence IDs are not unique')
    refset=set(ref_ids)
    object_ids={x['id'] for x in model.get('objects',[])}
    field_ids={obj['id']:{f['id'] for f in obj.get('fields',[])} for obj in model.get('objects',[])}
    for obj in model.get('objects',[]):
        if obj.get('id') not in ids: fail(errors,'object ID missing')
        for e in obj.get('evidence',[]):
            if e not in refset: fail(errors,f"unknown evidence ID {e}")
        for f in obj.get('fields',[]):
            if not f.get('evidence'): fail(errors,f"field {f.get('id')} has no field-level evidence")
            for e in f.get('evidence',[]):
                if e not in refset: fail(errors,f"field {f.get('id')} has unresolved evidence {e}")
    for group in ('guards','transitions','contracts'):
        for rec in model.get(group,[]):
            for e in rec.get('evidence',[]):
                if e not in refset: fail(errors,f"{group} {rec.get('id')} has unresolved evidence {e}")
    for tr in model['transitions']:
        if tr['object'] not in object_ids: fail(errors,f"transition {tr['id']} has unknown object")
        for ref in tr.get('key_field_writes',[]):
            if ref.get('object') not in object_ids or ref.get('field') not in field_ids.get(ref.get('object'),set()):
                fail(errors,f"transition {tr['id']} has unknown key field write {ref}")
        guard=tr.get('guard','')
        if guard.startswith('G-') and guard not in {g['id'] for g in model.get('guards',[])}:
            fail(errors,f"transition {tr['id']} references unknown guard {guard}")
    for r in refs:
        if r.get('status')!='RESOLVED': fail(errors,f"reference not resolved: {r.get('id')}"); continue
        p=r['path']; lo,hi=r['line_range']
        try: b=cmd(repo,'show',f"{a.target_sha}:{p}",text=False)
        except subprocess.CalledProcessError: fail(errors,f"source path absent in TARGET: {p}"); continue
        if hashlib.sha256(b).hexdigest()!=r['snapshot_sha256']: fail(errors,f"source SHA mismatch: {r['id']}")
        try: blob=cmd(repo,'rev-parse',f"{a.target_sha}:{p}").strip()
        except subprocess.CalledProcessError: blob=''
        if blob!=r.get('git_blob'): fail(errors,f"Git blob mismatch: {r['id']}")
        lines=b.decode('utf-8',errors='replace').splitlines()
        if lo<1 or hi<lo or hi>len(lines): fail(errors,f"line range invalid: {r['id']}")
        excerpt='\n'.join(f'{n}: {lines[n-1]}' for n in range(lo,hi+1))
        if excerpt!=r['excerpt']: fail(errors,f"support excerpt mismatch: {r['id']}")
    # Frozen snapshot consistency and the independently recomputed delta.
    delta=json.loads((out/'baseline/A_to_TARGET_delta.json').read_text())
    if delta.get('target_sha')!=a.target_sha: fail(errors,'delta TARGET_SHA mismatch')
    try:
        mb=cmd(repo,'show',f"{delta['as_is_artifact_commit']}:{delta['as_is_artifact_root']}/baseline/source_manifest.json",text=False)
        if hashlib.sha256(mb).hexdigest()!=delta['as_is_source_manifest_sha256']: fail(errors,'A manifest hash mismatch')
        manifest=json.loads(mb)
        recalculated=[]
        for item in manifest['files']:
            try: source=cmd(repo,'show',f"{a.target_sha}:{item['path']}",text=False); actual=hashlib.sha256(source).hexdigest(); state='same' if actual==item['sha256'] else 'changed'
            except subprocess.CalledProcessError: actual=None; state='missing_from_target'
            if state!='same': recalculated.append({'path':item['path'],'status':state,'A_sha256':item['sha256'],'TARGET_sha256':actual,'A_source_kind':item.get('source_kind')})
        if recalculated!=delta.get('changed_or_missing'): fail(errors,'A-to-TARGET source delta does not reproduce from frozen manifest')
        if len(manifest['files'])!=delta.get('as_is_source_count'): fail(errors,'A source count mismatch')
    except Exception as e: fail(errors,f'A manifest unavailable: {e}')
    status=cmd(repo,'status','--porcelain=v2','--untracked-files=all').splitlines()
    if status: fail(errors,'verification worktree is not clean')
    changed=cmd(repo,'diff','--name-only',a.target_sha,head).splitlines()
    outside=[p for p in changed if not p.startswith(str(OUT_REL)+'/')]
    if outside: fail(errors,'commit changes outside audit subtree: '+', '.join(outside))
    # All local links in HTML/Markdown resolve relative to their source file.
    link_files=list(out.rglob('*.md'))+list(out.rglob('*.html'))
    mdpat=re.compile(r'!?\[[^\]]*\]\(([^)]+)\)')
    for f in link_files:
        text=f.read_text(errors='replace')
        links=[]
        if f.suffix=='.html':
            parser=Links(); parser.feed(text); links=parser.values
        else: links=mdpat.findall(text)
        for link in links:
            link=link.strip().strip('<>')
            if not link or link.startswith(('https://','http://','mailto:','#','data:')): continue
            target=link.split('#',1)[0]
            if target and not (f.parent/target).resolve().exists(): fail(errors,f"broken local link in {f.relative_to(out)}: {link}")
    # Diagram transition references must resolve to canonical model IDs.
    transition_ids={t['id'] for t in model.get('transitions',[])}
    dot_sources=sorted((out/'diagrams/src').glob('*.dot'))
    if not dot_sources: fail(errors,'no Graphviz source diagrams')
    for dot in dot_sources:
        dot_text=dot.read_text(errors='replace')
        edge_ids=set(re.findall(r'\bT-[A-Z0-9-]+\b',dot_text))
        orphaned=edge_ids-transition_ids
        if orphaned: fail(errors,f"diagram {dot.name} references unknown transition IDs: {sorted(orphaned)}")
        svg=out/'diagrams/svg'/(dot.stem+'.svg')
        if not svg.exists(): fail(errors,f'missing rendered SVG for {dot.name}'); continue
        try: subprocess.run(['dot','-Tsvg',str(dot),'-o','/tmp/audit-state-contracts-check.svg'],check=True,capture_output=True,text=True)
        except Exception as e: fail(errors,f'Graphviz render failed for {dot.name}: {e}')
    # Compressed test/build logs are raw-byte preserved and hash checked.
    logs=json.loads((out/'validation/log_manifest.json').read_text())
    for entry in logs['logs']:
        p=out/'validation'/entry['file']
        try: zipped=p.read_bytes(); raw=gzip.decompress(zipped)
        except Exception as e: fail(errors,f"log decompression failed: {entry['file']}: {e}"); continue
        if hashlib.sha256(raw).hexdigest()!=entry['raw_sha256']: fail(errors,f"raw log hash mismatch: {entry['file']}")
        if hashlib.sha256(zipped).hexdigest()!=entry['gzip_sha256']: fail(errors,f"gzip hash mismatch: {entry['file']}")
    selection=json.loads((out/'tests/selection.json').read_text())
    if selection['target_sha']!=a.target_sha: fail(errors,'test selection TARGET_SHA mismatch')
    total=sum(x['tests'] for x in selection['test_groups'])
    spec=json.loads((out/'evidence/refs_spec.json').read_text())
    spec_ids=[x['id'] for x in spec['refs']]
    if len(spec_ids)!=len(set(spec_ids)): fail(errors,'evidence reference specification IDs are not unique')
    if set(spec_ids)!=refset: fail(errors,'resolved refs do not exactly match evidence reference specification')
    expected_fields=model.get('coverage',{}).get('fields',{}).get('scoped')
    actual_fields=sum(len(x.get('fields',[])) for x in model.get('objects',[]))
    if expected_fields!=actual_fields: fail(errors,'field coverage count does not match model')
    if model.get('coverage',{}).get('transitions',{}).get('documented')!=len(model.get('transitions',[])): fail(errors,'transition coverage count does not match model')
    coverage=model['coverage']
    result={'artifact_integrity':'PASS' if not errors else 'FAIL','target_source_integrity':'PASS' if not errors else 'FAIL','contract_coverage':'PARTIAL_AS_IS','tests':{'selected_groups':len(selection['test_groups']),'selected_cases':total,'status':'TEST_EXECUTED per evidence log; not full suite'},'performance_evidence':'NOT_CHARACTERIZED','publication':'NOT_CHECKED_BY_LOCAL_VERIFIER','audit_delivery':'PARTIAL','design_evidence':'PARTIAL_AS_IS','errors':errors,'warnings':warnings,'scope_coverage':coverage}
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return 1 if errors else 0
if __name__=='__main__': raise SystemExit(main())
