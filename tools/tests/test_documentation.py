"""Negative controls for the canonical documentation gate."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('check_documentation', Path(__file__).resolve().parents[1] / 'check_documentation.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

class DocumentationTests(unittest.TestCase):
    def check(self, text, source=None):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'docs/architecture').mkdir(parents=True)
            doc = root / 'docs/architecture/SYSTEM_DESIGN.md'
            doc.write_text(text)
            if source:
                path = root / source[0]
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(source[1])
            return MODULE.check_document(root, doc)

    def test_deleted_document_is_rejected(self):
        self.assertTrue(self.check('[old](../refactor/design/INDEX.md)'))

    def test_missing_inline_document_reference_is_rejected(self):
        self.assertTrue(self.check("Read `docs/refactor/design/INDEX.md` first."))

    def test_missing_source_citation_is_rejected(self):
        self.assertTrue(self.check('`src/missing.cpp:12`'))

    def test_out_of_range_citation_is_rejected(self):
        self.assertTrue(self.check('`src/a.cpp:2`', ('src/a.cpp', 'line\n')))

    def test_valid_source_and_external_link(self):
        self.assertEqual([], self.check('[source](../../src/a.cpp) `src/a.cpp:1` [ROS](https://docs.ros.org/)', ('src/a.cpp', 'line\n')))

    def test_valid_anchor_and_inline_code(self):
        self.assertEqual([], self.check('# Ownership\n[owner](#ownership) `not_a_path`'))

    def test_missing_anchor_is_rejected(self):
        self.assertTrue(self.check('# Ownership\n[owner](#missing)'))

if __name__ == '__main__':
    unittest.main()
