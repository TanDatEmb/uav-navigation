#!/usr/bin/env python3
"""Create a content-hash inventory of local raw experiment evidence."""
import hashlib
from pathlib import Path
ROOT=Path('/home/letandat/Dev/uav-navigation/.artifacts/runtime')
WORK=Path('/home/letandat/Dev/uav-navigation-experiment-runtime-observability-20260923')
B=WORK/'.artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c'
O=WORK/'artifacts/experiment_runtime_observability/20260923T041656Z-7da3e97c/EVIDENCE_INDEX.md'
rows=[
('A_OFF','external-mode-check-20260923T061558-15219',None),
('B_ON_NO_RECORDER','external-mode-check-20260923T062254-25436',None),
('C_ON_RECORDER','external-mode-check-20260923T062539-28804','C'),
('W1_CONTROL','external-mode-check-20260923T063510-38964','W1-fixed'),
('W2_800MS','external-mode-check-20260923T063803-42512','W2'),
('W3_550MS','external-mode-check-20260923T064024-45983','W3'),
('W4_3000MS','external-mode-check-20260923T064238-49492','W4'),
('W5_5000MS','external-mode-check-20260923T064439-52901','W5'),
('O1_F1_NOT_ARMED','external-mode-check-20260923T064712-56405','O1-F1'),
('O1_F2_NOT_ARMED','external-mode-check-20260923T065103-59893','O1-F2'),
('O1_R03','external-mode-check-20260923T065701-63505','O1-R03'),
('O1_R04','external-mode-check-20260923T065957-67359','O1-R04'),
('O1_R05','external-mode-check-20260923T070158-70759','O1-R05'),
('O1_R06','external-mode-check-20260923T070410-74150','O1-R06'),
('O1_R09','external-mode-check-20260923T071004-81913','O1-R09'),
('O1_R10','external-mode-check-20260923T071223-85315','O1-R10'),
]

def hash_file(path):
 d=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(8<<20),b''):d.update(chunk)
 return d.hexdigest()

def evidence_line(label,path,role):
 return f'| {label} | {role} | `{path}` | {path.stat().st_size} | `{hash_file(path)}` |'

lines=['# Evidence index','',
'All paths below are local, ignored raw data; their SHA-256 and byte sizes make them checkable. The committed compact CSVs are derived from the indexed normalized bags using `make_evidence_tables.py`. Product target `7da3e97c`; build switches and run provenance are in `PROVENANCE.md`. A/B/C and W/O1 episodes use the same nominal long-featured map profile and seed 23 unless the row says otherwise. No row is flight qualification.','',
'| Run | Role | Absolute path | Bytes | SHA-256 |','|---|---|---|---:|---|']
for label,session,bag in rows:
 s=ROOT/session
 for role,name in [('product_report','report.json'),('product_bag','rosbag/rosbag_0.db3'),('scenario','scenario.json')]:
  p=s/name
  if p.exists():lines.append(evidence_line(label,p,role))
 p=s/'audit_world_fault.jsonl'
 if p.exists():lines.append(evidence_line(label,p,'fault_gate'))
 if bag:
  for role,name in [('audit_bag',f'run-{bag}-recorder/hold_protocol_bag/hold_protocol_bag_0.mcap'),('audit_bag_metadata',f'run-{bag}-recorder/hold_protocol_bag/metadata.yaml'),('normalized_trace_integrity',f'run-{bag}-normalized/trace_integrity.json'),('topic_inventory',f'run-{bag}-recorder/topic_inventory.txt')]:
   p=B/name
   if p.exists():lines.append(evidence_line(label,p,role))
for role,name in [('off_build_manifest','off-build-manifest.json'),('on_build_manifest','on-build-manifest.json'),('on_build_after_gate_fix','on-build-after-gate-fix-manifest.json'),('on_build_before_o1_retry','on-build-before-o1-retry-manifest.json'),('px4_binary_hash','px4_binary.sha256'),('px4_checkout_status','px4_checkout_status.txt'),('off_test','off-test-result.log'),('on_test','on-test-result.log')]:
 p=B/name
 if p.exists():lines.append(evidence_line('BUILD',p,role))
lines+=['',
'`O1-R07` and `O1-R08` were excluded: the runner rejected a stale Release manifest before simulator topics existed, and the recorder timed out. Their logs are retained locally as invalid attempts. W1 initial `external-mode-check-20260923T063126-32806` was also excluded because the fault sidecar failed at Jazzy `use_sim_time` parameter declaration; W1-fixed is the evaluable control. These invalid attempts are not counted in evidence denominators.','',
'For reproducibility: validate each bag `metadata.yaml`, run `tools/audit_observability/normalize.py BAG OUT [--clock-proof RUN_PROOF]`, inspect `trace_integrity.json`, then run the committed compact-table generator. The clock proof IDs and sample counts are documented in `CLOCK_DOMAIN_CONTRACT.md` and the O2/O4 reports. SHA values for product bags cover the `.db3` payload; the metadata path is adjacent and includes topic inventory. PX4 source checkout was dirty before this experiment; `px4_binary.sha256` pins the executed binary independently of checkout HEAD.','']
O.write_text('\n'.join(lines))
print(len(rows),'runs',len(lines),'lines')
