"""Exercise root-baseline ROS selection with a fake colcon executable."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

class RootBaselineGateTests(unittest.TestCase):
    def test_same_root_remote_still_builds_every_package(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'tools').mkdir()
            shutil.copyfile(Path(__file__).resolve().parents[1] / 'gate.sh', root / 'tools/gate.sh')
            subprocess.run(['git', 'init', '-q'], cwd=root, check=True)
            subprocess.run(['git', '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'root', '--allow-empty'], cwd=root, check=True)
            subprocess.run(['git', 'update-ref', 'refs/remotes/origin/main', 'HEAD'], cwd=root, check=True)
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            colcon = bin_dir / 'colcon'
            colcon.write_text('#!/bin/sh\nif [ "$1" = list ]; then printf "alpha src/alpha cmake\\nbeta src/beta cmake\\n"; else printf "%s\\n" "$*" >> "$COLCON_CALL_LOG"; fi\n')
            colcon.chmod(0o755)
            env = dict(os.environ, PATH=str(bin_dir) + ':' + os.environ['PATH'], COLCON_CALL_LOG=str(root / 'calls'))
            env.pop('PACKAGES', None)
            result = subprocess.run(['bash', 'tools/gate.sh', 'ros'], cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertTrue((root / 'calls').exists(), 'root baseline skipped all ROS builds')
            self.assertIn('build --packages-select alpha beta', (root / 'calls').read_text())
            self.assertIn('GATE_V3_RESULT=PASS', result.stdout)
            self.assertIn('acceptance scope: product packages; upstream examples excluded', result.stdout)
            subprocess.run(['git', '-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'second', '--allow-empty'], cwd=root, check=True)
            (root / 'calls').write_text('')
            env['PACKAGES'] = 'alpha\nbeta'
            result = subprocess.run(['bash', 'tools/gate.sh', 'ros'], cwd=root, env=env, capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stdout + result.stderr)
            self.assertIn('build --packages-select alpha beta', (root / 'calls').read_text())


if __name__ == '__main__':
    unittest.main()
