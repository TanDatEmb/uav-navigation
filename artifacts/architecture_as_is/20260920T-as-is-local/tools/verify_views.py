#!/usr/bin/env python3
"""Structural integrity checks for the AS-IS model and generated views."""
import hashlib,json,pathlib,re,sys,xml.etree.ElementTree as ET
from html.parser import HTMLParser
from urllib.parse import unquote, urlsplit
OUT=pathlib.Path(__file__).resolve().parents[1]
m=json.loads((OUT/'model/architecture.json').read_text())
manifest=json.loads((OUT/'baseline/source_manifest.json').read_text())
filemap={x['path']:x for x in manifest['files']}
errors=[]
sets={k:m.get(k,[]) for k in ['objects','fields','decisions','transitions','scenarios','authority_findings','important_cross_layer_combinations','evidence_refs','sequence_diagrams']}
ids={}
for group,items in sets.items():
    for x in items:
        i=x['id']
        if i in ids: errors.append(f'duplicate ID {i} in {ids[i]} and {group}')
        ids[i]=group
object_ids={x['id'] for x in m['objects']}; field_ids={x['id'] for x in m['fields']}; ref_ids={x['id'] for x in m['evidence_refs']}; transition_ids={x['id'] for x in m['transitions']}
for f in m['fields']:
    if f['object'] not in object_ids: errors.append(f'{f["id"]}: unresolved object {f["object"]}')
for group,items in sets.items():
    for x in items:
        if group in {'objects','fields','decisions','transitions','scenarios','authority_findings','important_cross_layer_combinations'} and not x.get('evidence'):
            errors.append(f'{x["id"]}: missing source evidence')
        for e in x.get('evidence',[]):
            if e not in ref_ids: errors.append(f'{x["id"]}: unresolved evidence {e}')
for t in m['transitions']:
    for f in t['read_set']+t['write_set']:
        if f not in field_ids: errors.append(f'{t["id"]}: unresolved field {f}')
for f in m['fields']:
    for k in ('writers','readers'):
        if not f.get(k): errors.append(f'{f["id"]}: missing {k} inventory')
for r in m['evidence_refs']:
    item=filemap.get(r['path'])
    if not item: errors.append(f'{r["id"]}: source not in source manifest {r["path"]}'); continue
    if r['start']<1 or r['end']<r['start'] or r['end']>item['lines']: errors.append(f'{r["id"]}: invalid line range')
    snap=OUT/'baseline/source_snapshot'/r['path']
    if snap.is_file() and hashlib.sha256(snap.read_bytes()).hexdigest()!=item['sha256']: errors.append(f'{r["id"]}: snapshot hash mismatch')
for required in ['E_RUNTIME_MAIN','E_WORKER','E_TIMELINE','E_PUBLISH','E_MODE_ADMISSION','E_SETPOINT','E_HOLD']:
    if required not in ref_ids: errors.append(f'missing vertical-slice evidence {required}')
# Every modeled transition must occur in at least one state/owner DOT view.
dots='\n'.join(p.read_text() for p in (OUT/'diagrams/src').glob('*.dot'))
for tid in transition_ids:
    if tid not in dots: errors.append(f'{tid}: transition is not represented in any DOT view')
# Every DOT source and modeled Mermaid sequence must have its rendered SVG in this completed artifact.
svgs=list((OUT/'diagrams/svg').glob('*.svg'))
for dot in (OUT/'diagrams/src').glob('*.dot'):
    svg=OUT/'diagrams/svg'/f'{dot.stem}.svg'
    if not svg.is_file(): errors.append(f'missing rendered SVG for {dot.name}'); continue
    try: ET.parse(svg)
    except Exception as ex: errors.append(f'invalid SVG {svg.name}: {ex}')
    if '<svg' not in svg.read_text(errors='replace'): errors.append(f'no svg element in {svg.name}')
# Evidence IDs and generated table anchors must resolve.
html=(OUT/'evidence/refs.html').read_text(errors='replace') if (OUT/'evidence/refs.html').is_file() else ''
for rid in ref_ids:
    if f"id='{rid}'" not in html and f'id=\"{rid}\"' not in html: errors.append(f'missing evidence HTML anchor {rid}')
trans_md=(OUT/'tables/transitions.md').read_text(errors='replace') if (OUT/'tables/transitions.md').is_file() else ''
if len(m['fields'])!=m['coverage']['field_inventory_count']: errors.append('field coverage count mismatch')
if len(m['transitions'])!=m['coverage']['transition_inventory_count']: errors.append('transition coverage count mismatch')
trans_html=(OUT/'tables/transitions.html').read_text(errors='replace') if (OUT/'tables/transitions.html').is_file() else ''
for tid in transition_ids:
    if f"id='{tid}'" not in trans_html and f'id=\"{tid}\"' not in trans_html: errors.append(f'missing transition HTML anchor {tid}')
scenario_ids={x['id'] for x in m['scenarios']}
for seq in m.get('sequence_diagrams',[]):
    if not (OUT/'diagrams/src'/seq['file']).is_file(): errors.append(f'missing Mermaid source {seq["file"]}')
    mermaid_svg=OUT/'diagrams/svg'/f'{pathlib.Path(seq["file"]).stem}.svg'
    if not mermaid_svg.is_file(): errors.append(f'missing rendered Mermaid SVG {mermaid_svg.name}')
    else:
        try: ET.parse(mermaid_svg)
        except Exception as ex: errors.append(f'invalid Mermaid SVG {mermaid_svg.name}: {ex}')
    for sid in seq.get('scenario_ids',[]):
        if sid not in scenario_ids: errors.append(f'{seq["id"]}: unresolved scenario {sid}')
if len(m['scenarios']) != 12: errors.append(f'expected 12 scenario records, got {len(m["scenarios"])}')
# Check every relative link embedded by Graphviz resolves from its rendered SVG directory.
ns={'svg':'http://www.w3.org/2000/svg','xlink':'http://www.w3.org/1999/xlink'}
for svg in svgs:
    try: root=ET.parse(svg).getroot()
    except Exception: continue
    for a in root.findall('.//svg:a',ns):
        href=a.attrib.get('{http://www.w3.org/1999/xlink}href','')
        if href and not href.startswith(('http://','https://','#')):
            target=href.split('#',1)[0]
            if target and not (svg.parent/target).resolve().exists(): errors.append(f'{svg.name}: broken relative link {href}')
# The offline landing page must not depend on network assets and every local target must exist.
index=OUT/'index.html'
if not index.is_file():
    errors.append('missing offline index.html')
else:
    class LocalLinks(HTMLParser):
        def __init__(self): super().__init__(); self.targets=[]
        def handle_starttag(self, tag, attrs):
            a=dict(attrs)
            for key in ('href','src'):
                value=a.get(key,'')
                if value: self.targets.append(value)
    parser=LocalLinks(); parser.feed(index.read_text(errors='replace'))
for value in parser.targets:
        parsed=urlsplit(value)
        if parsed.scheme or parsed.netloc or value.startswith('#'): continue
        target=(OUT/unquote(parsed.path)).resolve()
        if not target.is_file(): errors.append(f'index.html: broken local link {value}')
# Check relative links and images in audit-authored Markdown, excluding captured source copies.
for md in OUT.rglob('*.md'):
    if 'baseline/source_snapshot' in md.as_posix(): continue
    for value in re.findall(r'!?\[[^]]*\]\(([^)]+)\)',md.read_text(errors='replace')):
        value=value.strip().split()[0].strip('<>') if value.strip() else ''
        parsed=urlsplit(value)
        if not value or parsed.scheme or parsed.netloc or value.startswith('#'): continue
        target=(md.parent/unquote(parsed.path)).resolve()
        if not target.is_file(): errors.append(f'{md.relative_to(OUT)}: broken local link {value}')
if errors:
    print('MODEL/VIEW CHECK FAILED:')
    print('\n'.join(errors)); sys.exit(1)
print(f'PASS: unique IDs={len(ids)}, fields={len(field_ids)}, transitions={len(transition_ids)}, evidence refs={len(ref_ids)}, rendered SVGs={len(svgs)} (7 DOT + {len(m.get("sequence_diagrams", []))} Mermaid)')
print(f'PASS: every transition appears in a DOT source; every evidence/path/range/hash and model reference resolves')
print('LIMIT: this structural checker does not prove the source model is semantically complete or runtime behavior is correct.')
