"""Generate compact audit diagrams as deterministic SVG artifacts."""
from pathlib import Path
from html import escape

DIAGRAMS={
 'legacy_authority.svg':('Legacy authority mirrors',[
  ('Core: goal / execution / world',90,70),('MissionController: gate / crossing',90,175),
  ('NavigationMode: mission / recovery / lease',440,175),('PX4 executor: Hold flags',440,280),
  ('WorldModel: latest map',90,280)],[(0,1),(0,2),(1,2),(2,3),(4,0)]),
 'shadow_authority.svg':('Shadow ownership proposal',[
  ('Core: MissionProgress + ExecutionAuthority',90,70),('WorldModel: immutable world',90,210),
  ('PX4 boundary: lease + Hold protocol',440,210),('Workers: immutable results only',440,70)],[(1,0),(3,0),(0,2)]),
 'pass_through_semantics.svg':('PASS_THROUGH: physical fact survives readiness',[
  ('Measured cursor + crossing witness',90,70),('Matching continuation witness',440,70),
  ('Accepted mission gate',265,210),('Route/reset/reverse invalidates',90,335)],[(0,2),(1,2),(3,0)]),
 'world_lease_handover.svg':('World certificate and command lease',[
  ('World source / certificate',90,70),('Active certified trajectory',440,70),
  ('100 ms adapter receive lease',440,210),('Hold request fences generation',440,335),
  ('Fresh world does not rearm old session',90,335)],[(0,1),(1,2),(2,3),(0,4),(3,4)]),
 'px4_hold_protocol.svg':('Hold facts are independent',[
  ('Hold request',90,70),('VehicleCommand ACK',440,70),
  ('ModeCompleted callback',90,210),('Fresh AUTO_LOITER status',440,210),
  ('Takeover / failsafe / deactivation',265,335)],[(0,1),(0,2),(0,3),(1,3),(2,3),(3,4)]),
}

def render(path:Path,title,nodes,edges):
    width,height=850,440
    def center(i):
        _,x,y=nodes[i];return x+155,y+35
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<defs><marker id="a" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10 Z" fill="#4361ee"/></marker></defs>',
           '<rect width="850" height="440" fill="#f8fafc"/>',
           f'<text x="30" y="35" font-family="sans-serif" font-size="22" font-weight="bold" fill="#14213d">{escape(title)}</text>']
    for a,b in edges:
        x1,y1=center(a);x2,y2=center(b)
        lines.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="#4361ee" stroke-width="2" marker-end="url(#a)"/>')
    for label,x,y in nodes:
        lines.append(f'<rect x="{x}" y="{y}" width="310" height="70" rx="9" fill="#e8efff" stroke="#8da9ff"/>')
        words=label.split();chunks=[];current=''
        for w in words:
            if len(current+' '+w)>33 and current: chunks.append(current);current=w
            else: current=(current+' '+w).strip()
        if current:chunks.append(current)
        for n,chunk in enumerate(chunks[:3]):
            lines.append(f'<text x="{x+155}" y="{y+31+n*18}" font-family="sans-serif" font-size="15" text-anchor="middle" fill="#14213d">{escape(chunk)}</text>')
    lines.append('</svg>');path.write_text('\n'.join(lines)+'\n')

if __name__=='__main__':
 import sys
 root=Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True)
 for name,(title,nodes,edges) in DIAGRAMS.items():render(root/name,title,nodes,edges)
