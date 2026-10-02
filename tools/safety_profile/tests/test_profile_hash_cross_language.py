import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.safety_profile.profile import load_profile


def main() -> int:
    executable, source = sys.argv[1:]
    cpp_hash = subprocess.check_output([executable], text=True).strip()
    python_hash = load_profile(Path(source)).sha256
    if cpp_hash != python_hash:
        print(f"C++/Python SafetyProfile hash mismatch: {cpp_hash} != {python_hash}")
        return 1
    print(f"C++ and Python SHA-256 match: {cpp_hash}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
