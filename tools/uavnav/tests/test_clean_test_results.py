"""Tests for tools.uavnav.clean_test_results (stale test-result cleanup before make test, D30)."""
import contextlib
import io
import os
import pathlib
import tempfile
import unittest

from tools.uavnav import clean_test_results as ctr


def touch(path, text="<x/>"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


class CleanTestResultsTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = pathlib.Path(tmp.name)
        self.base = self.root / "build"
        self.base.mkdir()

    def quiet_main(self, argv):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return ctr.main(argv)

    def test_removes_only_selected_package_results(self):
        x = touch(self.base / "a/test_results/a/x.gtest.xml")
        y = touch(self.base / "a/test_results/a/y.xunit.xml")
        t = touch(self.base / "a/Testing/20261001-0412/Test.xml")
        notes = touch(self.base / "a/test_results/a/notes.txt", "n")
        cache = touch(self.base / "a/CMakeCache.txt", "c")
        other = touch(self.base / "b/test_results/b/z.gtest.xml")
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ctr.clean(self.base, ["a"]), {"a": 3})
        for gone in (x, y, t):
            self.assertFalse(gone.exists(), gone)
        for kept in (notes, cache, other, self.base / "a/Testing/20261001-0412"):
            self.assertTrue(kept.exists(), kept)

    def test_rejects_path_like_package_names(self):
        keep = touch(self.base / "a/test_results/a/x.gtest.xml")
        for name in ("../a", "a/b", "", "."):
            with self.subTest(name=name):
                with self.assertRaises(ctr.InvalidPackageName):
                    ctr.clean(self.base, [name])
                self.assertTrue(keep.exists())

    def test_validates_every_name_before_deleting(self):
        keep = touch(self.base / "a/test_results/a/x.gtest.xml")
        with self.assertRaises(ctr.InvalidPackageName):
            ctr.clean(self.base, ["a", "../b"])
        self.assertTrue(keep.exists())

    def test_missing_package_dir_removes_nothing(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ctr.clean(self.base, ["c"]), {"c": 0})

    def test_does_not_follow_symlinks(self):
        outside = touch(self.root / "outside/q.xml")
        (self.base / "a").mkdir()
        os.symlink(self.root / "outside", self.base / "a/test_results")
        # A symlinked sub-directory and a symlinked file inside a real test_results dir.
        real = touch(self.base / "a2/test_results/a2/keep.txt", "k")
        os.symlink(self.root / "outside", self.base / "a2/test_results/linkdir")
        os.symlink(outside, self.base / "a2/test_results/link.xml")
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(ctr.clean(self.base, ["a"]), {"a": 0})
            self.assertEqual(ctr.clean(self.base, ["a2"]), {"a2": 0})
        self.assertTrue(outside.exists())
        self.assertTrue(real.exists())

    def test_cli_exit_codes(self):
        touch(self.base / "a/test_results/a/x.gtest.xml")
        self.assertEqual(self.quiet_main(["--build-base", str(self.base), "a"]), 0)
        self.assertEqual(self.quiet_main(["--build-base", str(self.base), "../a"]), 2)


if __name__ == "__main__":
    unittest.main()
