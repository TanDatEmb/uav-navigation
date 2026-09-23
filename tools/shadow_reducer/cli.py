"""python3 -m tools.shadow_reducer.cli --normalized-root DIR --output FILE"""
import argparse
import json
from pathlib import Path
from .replay import replay_all


def main():
    p=argparse.ArgumentParser(); p.add_argument('--normalized-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True); a=p.parse_args()
    if not a.normalized_root.is_dir(): p.error('normalized root missing')
    result=replay_all(a.normalized_root)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'runs':len(result['runs']),'totals':result['totals']},sort_keys=True))

if __name__=='__main__': main()
