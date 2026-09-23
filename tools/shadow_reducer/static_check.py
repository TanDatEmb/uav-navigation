"""Audit-only lexical guard; not a formal information-flow proof."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent
FORBIDDEN = (r'\brclpy\b', r'\brclcpp\b', r'\bpx4_ros2\b',
             r'\bschedulePx4Hold\(', r'\bNavigationCommand\(',
             r'\bNavigationGoal\(', r'\bcreate_publisher\(', r'\bpublish\(')

def check():
    checked=[]
    for path in ROOT.glob('*.py'):
        if path.name == 'static_check.py': continue
        content=path.read_text()
        found=[s for s in FORBIDDEN if re.search(s,content)]
        if found: raise AssertionError(f'{path}: forbidden product-authority token {found}')
        checked.append(path.name)
    return checked

if __name__=='__main__': print('PASS',', '.join(check()))
