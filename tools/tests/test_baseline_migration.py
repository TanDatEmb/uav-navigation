"""A pre-existing WIP deletion must not leave a retired doc in the publish index."""
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('migration', Path(__file__).resolve().parents[1] / 'verify_baseline_migration.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class PublishIndexTests(unittest.TestCase):
    def test_missing_retired_document_in_index_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            path = root / 'docs/refactor/old_prompt.md'
            path.parent.mkdir(parents=True)
            path.write_text('old authority\n')
            subprocess.run(['git', '-C', str(root), 'add', '--', str(path)], check=True)
            path.unlink()
            with self.assertRaises(ValueError):
                module.check_publish_index(root)

    def test_canonical_tracked_document_is_accepted(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            path = root / 'docs/ROADMAP.md'
            path.parent.mkdir(parents=True)
            path.write_text('current roadmap\n')
            subprocess.run(['git', '-C', str(root), 'add', '--', str(path)], check=True)
            module.check_publish_index(root)
