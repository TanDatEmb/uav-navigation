"""Tests for tools.uavnav.events (the JSONL event-log reader)."""
import contextlib
import io
import json
import os
import tempfile
import unittest

from tools.uavnav import events

LINE = ('{"t_steady_ns":2000000000,"t_ros_ns":0,"component":"supervisor","event":"Handover",'
        '"state_before":"RUNNING","state_after":"HANDED_OVER","reason":"NO_PATH_TIMEOUT","mission_id":1,'
        '"lio_epoch":1,"request_id":0,"bundle_id":0,"world_revision":0,"values":{"waited_s":15.0,"x":null}}')


def make_line(t_ns, component="supervisor", event="Handover", before="RUNNING",
              after="HANDED_OVER", reason="NO_PATH_TIMEOUT"):
    return json.dumps({
        "t_steady_ns": t_ns, "t_ros_ns": 0, "component": component, "event": event,
        "state_before": before, "state_after": after, "reason": reason,
        "mission_id": 1, "lio_epoch": 1, "request_id": 0, "bundle_id": 0,
        "world_revision": 0, "values": {}}, separators=(",", ":"))


GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "src", "core",
                      "uavnav_core", "test", "golden", "event_v1.jsonl")


class TempFileCase(unittest.TestCase):
    def write(self, text, binary=False):
        fd, path = tempfile.mkstemp(suffix=".jsonl")
        self.addCleanup(lambda: os.path.exists(path) and os.remove(path))
        with os.fdopen(fd, "wb") as f:
            f.write(text if binary else text.encode("utf-8"))
        return path


class LoadTest(TempFileCase):
    def test_load_parses_null_values(self):
        path = self.write(LINE + "\n")
        records = events.load(path)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["values"], {"waited_s": 15.0, "x": None})

    def test_load_reports_line_number_on_garbage(self):
        path = self.write(LINE + "\n{oops\n")
        with self.assertRaises(ValueError) as ctx:
            events.load(path)
        self.assertIn(":2:", str(ctx.exception))
        self.assertIn(path, str(ctx.exception))

    def test_load_skips_blank_lines_and_numbers_physically(self):
        path = self.write(LINE + "\n\n   \t\n{oops\n")
        with self.assertRaises(ValueError) as ctx:
            events.load(path)
        self.assertIn(":4:", str(ctx.exception))
        path2 = self.write("\n" + LINE + "\n  \n" + LINE + "\n")
        self.assertEqual(len(events.load(path2)), 2)

    def test_load_empty_file_is_empty_list(self):
        self.assertEqual(events.load(self.write("")), [])

    def test_load_accepts_uint64_max(self):
        big = LINE.replace('"mission_id":1', '"mission_id":18446744073709551615')
        self.assertEqual(events.load(self.write(big + "\n"))[0]["mission_id"], 2**64 - 1)

    def test_load_decodes_escapes_and_unicode(self):
        line = LINE.replace("NO_PATH_TIMEOUT", "a\\\"b\\n\\u0001é")
        self.assertEqual(events.load(self.write(line + "\n"))[0]["reason"], 'a"b\n\x01é')

    def test_load_missing_required_key_raises_naming_key(self):
        d = json.loads(LINE)
        del d["reason"]
        path = self.write(json.dumps(d) + "\n")
        with self.assertRaises(ValueError) as ctx:
            events.load(path)
        self.assertIn(":1:", str(ctx.exception))
        self.assertIn("reason", str(ctx.exception))

    def test_load_non_object_json_line_raises(self):
        for text in ("[1,2]", "42", "null", '"str"'):
            with self.subTest(text=text):
                with self.assertRaises(ValueError) as ctx:
                    events.load(self.write(text + "\n"))
                self.assertIn(":1:", str(ctx.exception))

    def test_load_rejects_wrong_field_types(self):
        for key, bad in (("t_steady_ns", "1"), ("t_steady_ns", True), ("component", 3),
                         ("values", [])):
            with self.subTest(key=key, bad=bad):
                d = json.loads(LINE)
                d[key] = bad
                with self.assertRaises(ValueError) as ctx:
                    events.load(self.write(json.dumps(d) + "\n"))
                self.assertIn(key, str(ctx.exception))

    def test_load_rejects_invalid_utf8_line(self):
        path = self.write(LINE.encode() + b"\n" + b'{"a":"\xff"}\n', binary=True)
        with self.assertRaises(ValueError) as ctx:
            events.load(path)
        self.assertIn(":2:", str(ctx.exception))

    def test_load_last_line_without_newline(self):
        self.assertEqual(len(events.load(self.write(LINE))), 1)

    def test_torn_last_line_strict_raises_and_skip_collects(self):
        torn = LINE[:40]
        path = self.write(LINE + "\n" + torn)
        with self.assertRaises(ValueError) as ctx:
            events.load(path)
        self.assertIn(":2:", str(ctx.exception))
        skipped = []
        records = events.load(path, skip_malformed=True, skipped=skipped)
        self.assertEqual(len(records), 1)
        self.assertEqual(len(skipped), 1)
        self.assertEqual(skipped[0][0], 2)
        self.assertIsInstance(skipped[0][1], str)

    def test_skip_malformed_collects_every_kind_with_line_numbers(self):
        d = json.loads(LINE)
        del d["event"]
        path = self.write("\n".join([
            LINE,            # 1 ok
            "{oops",         # 2 bad json
            "",              # 3 blank, silently ignored
            "[1]",           # 4 not an object
            json.dumps(d),   # 5 missing key
            LINE,            # 6 ok
        ]) + "\n")
        skipped = []
        records = events.load(path, skip_malformed=True, skipped=skipped)
        self.assertEqual(len(records), 2)
        self.assertEqual([n for n, _ in skipped], [2, 4, 5])
        self.assertIn("event", skipped[2][1])

    def test_skip_malformed_without_skipped_list_is_fine(self):
        path = self.write(LINE + "\n{oops\n")
        self.assertEqual(len(events.load(path, skip_malformed=True)), 1)

    def test_skip_malformed_is_keyword_only(self):
        with self.assertRaises(TypeError):
            events.load("x", True)  # type: ignore[misc]

    def test_load_missing_file_raises_oserror(self):
        with self.assertRaises(OSError):
            events.load(os.path.join(tempfile.gettempdir(), "uavnav-no-such-file.jsonl"))


class GoldenFileTest(unittest.TestCase):
    """The same file the C++ GoldenJsonl test pins; together they lock the wire format."""

    def test_golden_file_round_trip(self):
        records = events.load(GOLDEN)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0], {
            "t_steady_ns": 2000000000, "t_ros_ns": 1500000000, "component": "lio",
            "event": "LioStateChanged", "state_before": "RUNNING", "state_after": "STALE",
            "reason": "STALE_ODOMETRY", "mission_id": 7, "lio_epoch": 3, "request_id": 42,
            "bundle_id": 5, "world_revision": 9,
            "values": {"prefix_s": 1.5, "nan_value": None}})
        # Key order is part of the format.
        with open(GOLDEN, encoding="utf-8") as f:
            first = json.loads(f.readline(), object_pairs_hook=lambda pairs: [k for k, _ in pairs])
        self.assertEqual(first, ["t_steady_ns", "t_ros_ns", "component", "event", "state_before",
                                 "state_after", "reason", "mission_id", "lio_epoch", "request_id",
                                 "bundle_id", "world_revision", "values"])


class AnalysisTest(unittest.TestCase):
    def test_count_by_reason(self):
        rec = json.loads(LINE)
        self.assertEqual(events.count_by_reason([rec, dict(rec)]),
                         {("supervisor", "Handover", "NO_PATH_TIMEOUT"): 2})

    def test_count_by_reason_distinguishes_keys(self):
        a = json.loads(make_line(1))
        b = json.loads(make_line(2, reason="OTHER"))
        self.assertEqual(events.count_by_reason([a, b, a]),
                         {("supervisor", "Handover", "NO_PATH_TIMEOUT"): 2,
                          ("supervisor", "Handover", "OTHER"): 1})

    def test_timeline_sorted_and_formatted(self):
        late = json.loads(make_line(2_000_000_000, component="b"))
        early = json.loads(make_line(1_000_000_000, component="a"))
        lines = events.timeline([late, early])
        self.assertTrue(lines[0].startswith("1.000 "))
        self.assertEqual(lines[0], "1.000 a Handover RUNNING->HANDED_OVER NO_PATH_TIMEOUT")
        self.assertEqual(lines[1], "2.000 b Handover RUNNING->HANDED_OVER NO_PATH_TIMEOUT")

    def test_timeline_stable_for_equal_times(self):
        a = json.loads(make_line(5, component="first"))
        b = json.loads(make_line(5, component="second"))
        lines = events.timeline([a, b])
        self.assertIn(" first ", lines[0])
        self.assertIn(" second ", lines[1])

    def test_empty_inputs(self):
        self.assertEqual(events.count_by_reason([]), {})
        self.assertEqual(events.timeline([]), [])


class CliTest(TempFileCase):
    def run_cli(self, *argv):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = events.main(list(argv))
        return code, out.getvalue(), err.getvalue()

    def test_counts(self):
        path = self.write(LINE + "\n" + LINE + "\n")
        code, out, _ = self.run_cli(path, "--counts")
        self.assertEqual(code, 0)
        self.assertIn("2", out)
        self.assertIn("NO_PATH_TIMEOUT", out)

    def test_timeline_and_default_output(self):
        path = self.write(LINE + "\n")
        expected = "2.000 supervisor Handover RUNNING->HANDED_OVER NO_PATH_TIMEOUT\n"
        self.assertEqual(self.run_cli(path, "--timeline"), (0, expected, ""))
        self.assertEqual(self.run_cli(path), (0, expected, ""))

    def test_malformed_exits_2_without_traceback(self):
        path = self.write(LINE + "\n{oops\n")
        code, out, err = self.run_cli(path, "--counts")
        self.assertEqual(code, 2)
        self.assertEqual(out, "")
        self.assertIn(":2:", err)
        self.assertNotIn("Traceback", err)

    def test_skip_malformed_reports_count_on_stderr(self):
        path = self.write(LINE + "\n{oops\n[1]\n")
        code, out, err = self.run_cli(path, "--timeline", "--skip-malformed")
        self.assertEqual(code, 0)
        self.assertEqual(out.count("\n"), 1)
        self.assertIn("skipped 2 malformed line(s)", err)

    def test_missing_file_is_nonzero_with_message(self):
        missing = os.path.join(tempfile.gettempdir(), "uavnav-no-such-file.jsonl")
        code, out, err = self.run_cli(missing)
        self.assertNotEqual(code, 0)
        self.assertIn(missing, err)
        self.assertNotIn("Traceback", err)

    def test_counts_and_timeline_are_exclusive(self):
        path = self.write(LINE + "\n")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as ctx:
            events.main([path, "--counts", "--timeline"])
        self.assertEqual(ctx.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
