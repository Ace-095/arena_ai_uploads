#!/usr/bin/env python3
"""pi_agent - let a remote operator drive this Pi through GitHub Issues.

WHY THIS EXISTS
===============
The machine that wrote this repo cannot reach a private LAN address (it has no
route to 192.168.x.x, and its outbound access is an allowlist that excludes
tunnels), so there is no way for it to open a shell here. This closes that gap
without opening an inbound port on the aircraft's network.

The Pi polls GitHub - outbound HTTPS only - for issues labelled `pi:job`. An
issue body is a one-line command. The agent runs it, posts the output back as a
comment and labels the issue `pi:done`. The operator reads the comment with
`gh issue view`. That is the entire protocol, and it is deliberately boring.

THE PART THAT MATTERS IS AUTHORISATION
=====================================
This repository is PUBLIC. Anyone on the internet can open an issue, so
"execute whatever a comment says" would hand a stranger the aircraft's
computer. Three independent controls stand in the way, and all three must fail
at once for a stranger to run anything:

1. **GitHub authenticates the author.** A job runs only if the issue author is
   in an explicit login allowlist (default: the repository owner). Nothing in
   the comment body can forge that - it is server-side truth from the API.
2. **Commands are not shell.** There is no interpreter here. The body selects
   an ACTION from a fixed registry and supplies typed PARAMETERS. No free-form
   string ever reaches a command line; `; rm -rf /` is not escaped, it simply
   is not a valid parameter.
3. **No action can actuate anything.** The registry contains no arming, no mode
   change, no parameter write, no config write, no fence upload. A test fails
   the suite if such an action is ever added.

Anything that opens the camera or the flight-controller serial port is classed
`hardware` and is refused unless the operator explicitly says `hardware=yes`
AND `mission_pi` is not running (it holds those devices; two owners on one CSI
camera or one serial port is a support call, not a measurement).

WHY NOT A SELF-HOSTED GITHUB ACTIONS RUNNER
===========================================
That is the established way to run jobs on your own machine, and it is wrong
here: GitHub's own documentation says only to use self-hosted runners with
private repositories, because anyone can fork a public repository and get code
executed on the runner. This repository is public, so a runner would be a
permanent, documented remote-code-execution path onto this Pi. A polling agent
that executes a three-entry allowlist is a much smaller thing to get wrong.

HONEST LIMITS
=============
* It is not interactive and it is not a shell. New action = new code + test.
* Latency is the poll interval (default 20 s), plus however long the job takes.
* It cannot do anything physical: no plugging in, no moving a sheet of paper,
  no holding the aircraft. Anything with hands in it is still yours.
* A job that needs the camera needs nobody else using the camera.
* `--dry-run` proves the plumbing without executing anything.

This tool is READ-ONLY with respect to the aircraft by construction: every
action in the registry is observation or test.

Usage
-----
    # one pass, then exit - good for a cron entry or a first try
    python3 tools/pi_agent.py --once

    # keep watching, 20 s between polls
    python3 tools/pi_agent.py --loop --interval 20

    # maybe the robots are wrong. read it through:
    python3 tools/pi_agent.py --once --dry-run

Environment
-----------
    PI_AGENT_TOKEN   GitHub token with "Issues: Read and write" plus "Metadata:
                     read" on THIS REPOSITORY ONLY - a fine-grained personal
                     access token. Use --token-env to name a different variable.
                     The token is never printed, logged or posted.

                     Read and write on issues is genuinely required, not a
                     convenience: a token that can create the issue but not
                     label or comment on it makes GitHub drop the label in
                     silence, and the job is then never picked up. `--queue`
                     checks the label stuck before claiming success.
    PI_AGENT_REPO    owner/name, if --repo is not given.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
import traceback
import urllib.error
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                     # .../mission_pi
PY = sys.executable or "python3"
FENCE = "```"

JOB_LABEL = "pi:job"
DONE_LABEL = "pi:done"
REFUSED_LABEL = "pi:refused"
LABELS = (JOB_LABEL, DONE_LABEL, REFUSED_LABEL)

# GitHub rejects comments over 65536 chars; leave room for the wrapper prose.
COMMENT_BUDGET = 55000
# An inline frame must survive base64 (+33%) and the comment wrapper.
IMAGE_BUDGET = 40000

API = "https://api.github.com"
UA = "pi-agent/1.0 (+https://github.com/)"


# ---------------------------------------------------------------------------
# redaction - tested, because a token in a public issue comment is forever
# ---------------------------------------------------------------------------
_SECRETS = []


def register_secret(value):
    """Never let this string reach stdout, a log or a comment."""
    if value and len(str(value)) >= 8:
        _SECRETS.append(str(value))


def redact(text):
    if text is None:
        return ""
    text = str(text)
    for s in _SECRETS:
        text = text.replace(s, "<redacted>")
    # belt and braces: tokens in output we did not register (e.g. nested tools)
    text = re.sub(r"(gh[pousr]_[A-Za-z0-9_]{16,})", "<redacted>", text)
    text = re.sub(r"(github_pat_[A-Za-z0-9_]{16,})", "<redacted>", text)
    text = re.sub(r"(?i)\b(token|authorization)\b\s*[:=]\s*(?:bearer\s+)?\S{6,}",
                  r"\1=<redacted>", text)
    text = re.sub(r"(?i)\bbearer\s+\S{6,}", "bearer <redacted>", text)
    return text


# ---------------------------------------------------------------------------
# actions
# ---------------------------------------------------------------------------
# Parameter types. A parameter is only ever one of these - never a string that
# is passed through to a shell, so there is nothing to escape.
FLOAT = "float"
ENUM = "enum"
# A parameter default of OMIT means "optional flag": when the operator does not
# supply it, the flag is left off entirely rather than sent with a dummy value.
OMIT = object()


class Action(object):
    """One thing a remote operator is allowed to ask for.

    kind: "read_only"  - observation only, runs without further ceremony
          "hardware"   - opens the camera or the FC serial port, so it needs
                         an explicit hardware=yes and an idle mission_pi
    """

    def __init__(self, name, kind, argv=None, handler=None, params=None,
                 timeout=120, summary="", needs="", max_timeout=None):
        self.name = name
        self.kind = kind
        self.argv = argv
        self.handler = handler
        self.params = params or {}
        self.timeout = timeout
        self.summary = summary
        self.needs = needs
        self.max_timeout = max_timeout

    def check(self, values):
        """Validate parameters WITHOUT building anything. -> error string or None.

        Extracted from build() for a reason found by reading this file again:
        handler actions (capture) were dispatched without touching build(), so
        their parameters were never validated at all - an enum parameter could
        carry any string. Every action now passes through here first.
        """
        for key, spec in self.params.items():
            kind, _flag, default = spec[0], spec[1], spec[2]
            got = values.get(key)
            if got is None:
                if default is None:
                    return "missing required parameter `%s`" % key
                continue                      # default (or OMIT) is fine
            if kind == FLOAT:
                try:
                    num = float(got)
                except ValueError:
                    return "parameter `%s` must be a number, got %r" % (key, got)
                if num != num or num in (float("inf"), float("-inf")):
                    return "parameter `%s` must be a finite number" % key
                if len(spec) > 3 and spec[3]:
                    lo, hi = spec[3]
                    if not (lo <= num <= hi):
                        return ("parameter `%s` must be between %s and %s"
                                % (key, lo, hi))
            elif kind == ENUM:
                if got not in (spec[3] or ()):
                    return "parameter `%s` must be one of %s" % (
                        key, ", ".join(sorted(spec[3] or ())))
            else:
                return "internal: unknown parameter type for `%s`" % key
        leftovers = sorted(set(values) - set(self.params) - {"hardware"})
        if leftovers:
            return ("unknown parameter(s): %s - this agent takes a fixed list, "
                    "it is not a shell" % ", ".join(leftovers))
        return None

    def build(self, values):
        """-> (argv, None) or (None, human-readable refusal)."""
        err = self.check(values)
        if err:
            return None, err
        argv = list(self.argv) if self.argv else []
        for key, spec in self.params.items():
            kind, flag, default = spec[0], spec[1], spec[2]
            got = values.get(key)
            if got is None:
                got = default
            if got is OMIT or got is None:
                continue                      # optional flag, not supplied
            if kind == FLOAT:
                argv += [str(flag), ("%g" % float(got))]
            else:
                argv += [str(flag), str(got)]
        return argv, None


def _act(name, kind, argv, **kw):
    kw["argv"] = argv
    return Action(name, kind, **kw)


def _capture_handler(args, params):
    """Grab one frame and return a small JPEG the operator can actually look at.

    Returns (text, jpeg_bytes_or_None). Imported lazily so the agent still runs
    on a machine without numpy/cv2 - it just reports that it cannot.
    """
    role = params.get("role", "bottom")
    try:
        import cv2
        import numpy as np
    except Exception as e:
        return "cannot capture on this machine: %s: %s" % (type(e).__name__, e), None
    sys.path.insert(0, ROOT)
    sys.path.insert(0, HERE)
    try:
        import pi_field_report as P
        cfg = P._cfg(os.path.join(ROOT, "config.yaml"))
        rig, cam = P._open_camera(cfg, role)
        if cam is None:
            return "no camera detected for role %r" % role, None
        try:
            frames, ms = P._capture_into(role, cam, seconds=3.0, want=3)
        finally:
            try:
                cam.stop()
            except Exception:
                pass
            try:
                rig.stop_all()
            except Exception:
                pass
        if not frames:
            return "camera produced no frames for role %r" % role, None
        # Sharpest of the three: a motion-blurred frame proves nothing about FOV.
        best, best_score = frames[0], -1.0
        for f in frames:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
            score = float(cv2.Laplacian(g, cv2.CV_64F).var())
            if score > best_score:
                best, best_score = f, score
        h, w = best.shape[:2]
        lines = ["captured %d frames for role %s" % (len(frames), role),
                 "frame size: %dx%d" % (w, h),
                 "latest() ms: %s" % ("/".join("%.0f" % x for x in ms)),
                 "sharpest frame Laplacian variance: %.0f" % best_score]
        # Only shrink if we must; text always survives, the picture may not.
        jpeg = None
        for max_dim in (900, 720, 560, 420, 320):
            for q in (80, 65, 50, 35):
                scale = min(1.0, float(max_dim) / max(w, h))
                out = best if scale >= 1.0 else cv2.resize(
                    best, (int(w * scale), int(h * scale)),
                    interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", out,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), q])
                if not ok:
                    continue
                if len(buf) * 4 / 3 <= IMAGE_BUDGET:
                    jpeg, chosen = buf.tobytes(), (max_dim, q)
                    break
            if jpeg is not None:
                break
        if jpeg is None:
            lines.append("frame too large to inline as a comment - not attached")
        else:
            lines.append("attached downscaled JPEG: %d bytes (limit %d, "
                         "quality %d)" % (len(jpeg), IMAGE_BUDGET, chosen[1]))
        return "\n".join(lines), jpeg
    except Exception:
        return ("capture failed:\n" + redact(traceback.format_exc(limit=4))), None


# The whole surface a remote operator has. Read it top to bottom: there is no
# arming, no mode change, no parameter or config write, no fence upload, no
# arbitrary command. That is the design, not an oversight.
REGISTRY = {}


def _help_handler(action, params):
    return help_text()


def _register(*actions):
    for a in actions:
        REGISTRY[a.name] = a


_register(
    _act("help", "read_only", None, handler=_help_handler, timeout=10,
         summary="list the actions this agent will perform"),
    _act("scout", "read_only",
         [PY, os.path.join(HERE, "pi_field_report.py"), "--no-capture"],
         timeout=600,
         summary="the read-only field survey (no camera, no FC)"),
    _act("cameras", "read_only",
         [PY, os.path.join(HERE, "list_cams.py")], timeout=120,
         summary="enumerate cameras and the role each is assigned"),
    _act("hailo", "read_only",
         [PY, os.path.join(HERE, "hef_info.py")], timeout=180,
         summary="Hailo platform, devices and compiled model inventory"),
    _act("tests", "read_only",
         ["bash", os.path.join(HERE, "run_tests.sh")], timeout=2700,
         summary="the whole mission_pi suite, on the Pi, for real (slow)"),
    _act("fov", "hardware", [PY, os.path.join(HERE, "pi_field_report.py"),
                             "--sec", "fov"], timeout=300,
         summary="MEASURE the real field of view from a sheet of known size",
         params={"dist_m": (FLOAT, "--dist-m", None, (0.2, 20.0)),
                 "panel_mm": (FLOAT, "--panel-mm", None, (50.0, 5000.0)),
                 # optional: omitting it skips VFOV and the aspect check
                 "panel_mm_h": (FLOAT, "--panel-mm-h", OMIT, (50.0, 5000.0)),
                 # OMIT, not "bottom": pi_field_report already defaults to
                 # bottom, so the agent should not insist on a parameter the
                 # tool treats as optional. Explicit role=front still works.
                 "role": (ENUM, "--role", OMIT, ("bottom", "front"))},
         needs="a white sheet flat on a darker floor, centred under the lens, "
               "and a tape measure from lens to sheet (dist_m)"),
    _act("fence", "hardware",
         [PY, os.path.join(HERE, "fc_probe.py"), "--fence-check"], timeout=180,
         summary="read the Pixhawk's fence and compare it to the export",
         needs="mission_pi stopped (it holds the serial port), vehicle disarmed"),
    _act("verify", "hardware",
         [PY, os.path.join(HERE, "pi_field_report.py"), "--with-fc"],
         timeout=600,
         summary="the survey plus the Pixhawk: firmware, capabilities, fence",
         needs="mission_pi stopped, vehicle disarmed"),
    _act("capture", "hardware", None, handler=_capture_handler, timeout=180,
         summary="one frame from the camera, attached as a JPEG",
         params={"role": (ENUM, "--role", "bottom", ("bottom", "front"))},
         needs="nobody else using the camera"),
)


def help_text():
    lines = ["Actions this agent will perform:", ""]
    for name in sorted(REGISTRY):
        a = REGISTRY[name]
        tag = "hardware" if a.kind == "hardware" else "read-only"
        lines.append("* `%s` (%s, %ds) - %s" % (name, tag, a.timeout, a.summary))
        if a.params:
            for k, s in sorted(a.params.items()):
                if s[0] == FLOAT:
                    lines.append("    - `%s` = number" % k)
                else:
                    lines.append("    - `%s` = one of %s" % (k, ", ".join(s[3])))
        if a.needs:
            lines.append("    - needs: %s" % a.needs)
    lines += ["", "Anything not on this list is refused. There is no shell here.",
              "Hardware actions also need `hardware=yes` in the command."]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# job parsing
# ---------------------------------------------------------------------------
class Job(object):
    def __init__(self, issue, action, params, hardware_ok, error=None):
        self.issue = issue
        self.action = action
        self.params = params
        self.hardware_ok = hardware_ok
        self.error = error

    @property
    def number(self):
        return self.issue["number"]


TOKEN_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,30}$")


def parse_job(issue):
    """Parse an issue body into a Job.

    Grammar, whole thing:

        /pi <action> [key=value ...] [hardware=yes]

    Unknown actions and undecodable parameters are carried on the Job as
    `error` so the agent can answer with a helpful refusal instead of silence.
    """
    body = (issue.get("body") or "").strip()
    first = body.splitlines()[0].strip() if body else ""
    if not first.startswith("/pi"):
        return Job(issue, None, {}, False,
                   error="I only understand commands that start with `/pi`.")
    tokens = first.split()[1:]
    if not tokens:
        return Job(issue, None, {}, False, error="`/pi` with no action.")
    action = tokens[0].lower()
    params, hardware_ok = {}, False
    for tok in tokens[1:]:
        if "=" not in tok:
            return Job(issue, action, {}, False,
                       error="`%s` is not `key=value` - this agent takes typed "
                             "parameters, not free-form text." % tok)
        key, _, value = tok.partition("=")
        key = key.strip().lower()
        if not TOKEN_RE.match(key):
            return Job(issue, action, {}, False,
                       error="parameter name `%s` is not a name." % key[:40])
        if key == "hardware":
            hardware_ok = value.strip().lower() in ("yes", "true", "1", "y")
            continue
        params[key] = value.strip()
    if action not in REGISTRY:
        near = [n for n in REGISTRY if n.startswith(action[:3])]
        hint = (" Did you mean %s?" % ", ".join("`%s`" % n for n in near)
                if near else "")
        return Job(issue, action, params, hardware_ok,
                   error="`%s` is not an action this agent performs.%s" % (action, hint))
    return Job(issue, action, params, hardware_ok)


# ---------------------------------------------------------------------------
# guards
# ---------------------------------------------------------------------------
def mission_pi_running(ps_output=None):
    """True if mission_pi holds the camera / serial port right now."""
    if ps_output is None:
        try:
            ps_output = subprocess.run(
                ["ps", "-eo", "args"], stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, timeout=10,
            ).stdout.decode("utf-8", "replace")
        except Exception:
            return False
    for line in ps_output.splitlines():
        line = line.strip()
        if not line or "pi_agent" in line or "ps -eo" in line:
            continue
        if re.search(r"\bmission_pi/(main\.py|mission\.py)\b", line) or \
           re.search(r"(^|\s|/)main\.py(\s|$)", line):
            return True
    return False


# ---------------------------------------------------------------------------
# execution
# ---------------------------------------------------------------------------
class Result(object):
    def __init__(self, argv, rc, out, seconds, timed_out=False, refused=None,
                 jpeg=None):
        self.argv = argv
        self.rc = rc
        self.out = out
        self.seconds = seconds
        self.timed_out = timed_out
        self.refused = refused
        self.jpeg = jpeg


def run_argv(argv, timeout, cwd=None, env=None):
    """Run, never raise, always reap the whole process group.

    start_new_session puts the child in its own group so a timeout kills the
    grandchildren too - `run_tests.sh` spawns many, and killing only the shell
    would leave a test run behind, holding cameras and writing to the repo.
    """
    t0 = time.time()
    try:
        p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             cwd=cwd or ROOT, env=env, start_new_session=True)
    except Exception as e:
        return Result(argv, 127, "%s: %s" % (type(e).__name__, e),
                      time.time() - t0)
    try:
        out, _ = p.communicate(timeout=timeout)
        return Result(argv, p.returncode, out.decode("utf-8", "replace"),
                      time.time() - t0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            time.sleep(1.5)
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
        try:
            out, _ = p.communicate(timeout=10)
        except Exception:
            out = b""
        return Result(argv, 124,
                      out.decode("utf-8", "replace") +
                      "\n[killed after %ds]" % timeout,
                      time.time() - t0, timed_out=True)
    except Exception as e:
        try:
            p.kill()
        except Exception:
            pass
        return Result(argv, 125, "%s: %s" % (type(e).__name__, e),
                      time.time() - t0)


def execute(job, base_env=None, ps_output=None):
    """Run one Job under every guard. Never raises."""
    action = REGISTRY.get(job.action)
    env = dict(base_env if base_env is not None else os.environ)
    if action is None:
        return Result([], 2, job.error or "no such action", 0.0,
                      refused=job.error)
    if job.error:
        return Result([], 2, job.error, 0.0, refused=job.error)
    param_err = action.check(job.params)
    if param_err:
        return Result([], 2, param_err, 0.0, refused=param_err)
    if action.kind == "hardware" and not job.hardware_ok:
        msg = ("`%s` opens hardware (camera or flight-controller serial port). "
               "Re-send it with `hardware=yes` on the end so it cannot happen "
               "by accident." % action.name)
        if action.needs:
            msg += "\nIt needs: %s" % action.needs
        return Result([], 2, msg, 0.0, refused=msg)
    if action.kind == "hardware" and mission_pi_running(ps_output):
        msg = ("`mission_pi` is running and holds the camera / serial port. "
               "Stop it first, then re-send." % ())
        return Result([], 2, msg, 0.0, refused=msg)
    if action.handler is not None:
        t0 = time.time()
        try:
            got = action.handler(action, job.params)
        except Exception:
            return Result([action.name], 1, redact(traceback.format_exc(limit=6)),
                          time.time() - t0)
        if isinstance(got, (tuple, list)):
            seq = list(got) + [None]
            text, jpeg = seq[0], seq[1]
        else:
            text, jpeg = got, None
        rc = 0 if not str(text).lower().startswith(
            ("cannot", "no camera", "capture failed")) else 3
        return Result([action.name] + (["(%s)" % job.params] if job.params else []),
                      rc, text, time.time() - t0, jpeg=jpeg)
    argv, err = action.build(job.params)
    if err:
        return Result([], 2, err, 0.0, refused=err)
    return run_argv([a for a in argv if a], action.timeout, env=env)


# ---------------------------------------------------------------------------
# rendering
# ---------------------------------------------------------------------------
def render_comment(job, res, meta):
    """The reply. Fixed shape so it is skimmable over `gh issue view`."""
    lines = []
    if res.refused:
        lines.append("### pi-agent: refused")
    elif res.rc == 0:
        lines.append("### pi-agent: ok")
    else:
        lines.append("### pi-agent: exit %s" % res.rc)
    bits = ["**action** `%s`" % (job.action or "-"),
            "**host** %s" % meta["host"],
            "**exit** %s" % res.rc,
            "**took** %.1f s" % res.seconds,
            "**read-only** %s" % ("yes" if _is_read_only(job.action) else
                                  "no - it opened hardware")]
    lines.append(" | ".join(bits))
    if res.timed_out:
        lines.append("")
        lines.append("> Timed out and was killed, process group and all.")
    lines += ["", "<details><summary>what ran</summary>", "",
              FENCE + "text", meta["cmdline"], FENCE, "", "</details>"]
    body = res.out or "(no output)"
    if len(body) > COMMENT_BUDGET:
        keep = COMMENT_BUDGET - 200
        head, tail = body[:keep * 2 // 3], body[-keep // 3:]
        body = ("%s\n\n... [%d bytes cut from the middle - the full output is in "
                "the agent's local log: %s] ...\n\n%s"
                % (head, len(res.out) - keep, meta.get("log_path", "see --log"),
                   tail))
    lines += ["", "<details><summary>output</summary>", "", FENCE + "text",
              redact(body), FENCE, "", "</details>"]
    if res.jpeg:
        import base64
        lines += ["", "<details><summary>frame (base64 JPEG - decode to view)"
                      "</summary>", "", FENCE + "text",
                  base64.b64encode(res.jpeg).decode("ascii"), FENCE, "",
                  "</details>"]
    lines += ["", "---", "", meta["footer"]]
    return "\n".join(lines)


def _is_read_only(action_name):
    a = REGISTRY.get(action_name or "")
    return bool(a) and a.kind == "read_only"


# ---------------------------------------------------------------------------
# transports
# ---------------------------------------------------------------------------
class GitHubTransport(object):
    """Thin urllib client. Conditional requests keep polling nearly free:
    GitHub does not count a 304 against the rate limit."""

    def __init__(self, repo, token, timeout=30, verbose=False):
        self.repo = repo
        self.token = token
        self.timeout = timeout
        self.verbose = verbose
        self._etags = {}
        register_secret(token)

    def _call(self, method, path, payload=None, etag_key=None, raw=False):
        url = path if path.startswith("http") else API + path
        data = None
        headers = {"Accept": "application/vnd.github+json",
                   "User-Agent": UA,
                   "X-GitHub-Api-Version": "2022-11-28"}
        if self.token:
            headers["Authorization"] = "Bearer %s" % self.token
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if etag_key and self._etags.get(etag_key):
            headers["If-None-Match"] = self._etags[etag_key]
        req = urllib.request.Request(url, data=data, headers=headers,
                                     method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                body = r.read()
                if etag_key and r.headers.get("ETag"):
                    self._etags[etag_key] = r.headers["ETag"]
                if raw:
                    return r.status, body
                return r.status, (json.loads(body.decode("utf-8"))
                                  if body else None)
        except urllib.error.HTTPError as e:
            if e.code == 304:
                return 304, None
            detail = ""
            try:
                detail = e.read().decode("utf-8", "replace")[:400]
            except Exception:
                pass
            raise RuntimeError("GitHub %s %s -> %s %s"
                               % (method, url, e.code, redact(detail)))
        except Exception as e:
            raise RuntimeError("GitHub %s %s -> %s: %s"
                               % (method, url, type(e).__name__, redact(str(e))))

    # -- interface the agent uses -------------------------------------------
    def whoami(self):
        _s, d = self._call("GET", "/user")
        return (d or {}).get("login")

    def repo_owner(self):
        _s, d = self._call("GET", "/repos/%s" % self.repo)
        return (d or {}).get("owner", {}).get("login")

    def list_jobs(self):
        """Open issues carrying the job label. PRs are not issues here."""
        path = ("/repos/%s/issues?state=open&labels=%s&per_page=20"
                "&sort=created&direction=asc"
                % (self.repo, urllib.parse.quote(JOB_LABEL)))
        status, data = self._call("GET", path, etag_key="jobs")
        if status == 304:
            return []
        out = []
        for it in (data or []):
            if "pull_request" in it:
                continue
            out.append(it)
        return out

    def ensure_labels(self):
        have = set()
        try:
            _s, d = self._call("GET", "/repos/%s/labels?per_page=100" % self.repo)
            have = {x["name"] for x in (d or [])}
        except Exception:
            pass
        colours = {JOB_LABEL: "0e8a16", DONE_LABEL: "c5def5",
                   REFUSED_LABEL: "d93f0b"}
        for name in LABELS:
            if name in have:
                continue
            try:
                self._call("POST", "/repos/%s/labels" % self.repo,
                           {"name": name, "color": colours[name],
                            "description": "pi_agent remote link"})
            except Exception as e:
                if self.verbose:
                    print("label %s: %s" % (name, redact(str(e))))

    def comment(self, number, body):
        for i in range(0, len(body), 60000):        # never exceed the API cap
            self._call("POST", "/repos/%s/issues/%s/comments"
                       % (self.repo, number), {"body": body[i:i + 60000]})

    def create_issue(self, title, body, labels=(JOB_LABEL,)):
        """Queue a job. Returns (url, number, labels_actually_applied).

        The third value exists because of something measured, not imagined: a
        token that may create issues but not apply labels to them makes GitHub
        create the issue and SILENTLY DROP the label, reporting success. Verified
        against the live API - the issue appeared with zero labels, no error, and
        no timeline event. A queued job that silently is not queued is worse than
        a refusal, so this re-reads the issue and reports what actually stuck.
        """
        _s, d = self._call("POST", "/repos/%s/issues" % self.repo,
                           {"title": title, "body": body, "labels": list(labels)})
        url = (d or {}).get("html_url")
        number = (d or {}).get("number")
        got = [(l.get("name") if isinstance(l, dict) else l)
               for l in ((d or {}).get("labels") or [])]
        if number is not None and not got:
            try:
                _s2, d2 = self._call("GET", "/repos/%s/issues/%s"
                                     % (self.repo, number))
                got = [(l.get("name") if isinstance(l, dict) else l)
                       for l in ((d2 or {}).get("labels") or [])]
            except Exception:
                pass
        return url, number, got

    def add_labels(self, number, labels):
        self._call("POST", "/repos/%s/issues/%s/labels"
                   % (self.repo, number), {"labels": list(labels)})

    def remove_label(self, number, label):
        self._call("DELETE", "/repos/%s/issues/%s/labels/%s"
                   % (self.repo, number, urllib.parse.quote(label)))

    def create_label(self, name, color="ededed", description=""):
        self._call("POST", "/repos/%s/labels" % self.repo,
                   {"name": name, "color": color, "description": description})

    def delete_label(self, name):
        self._call("DELETE", "/repos/%s/labels/%s"
                   % (self.repo, urllib.parse.quote(name)))

    def list_issues(self, limit=5):
        _s, d = self._call("GET", "/repos/%s/issues?state=open&per_page=%d"
                           % (self.repo, limit))
        return [i for i in (d or []) if "pull_request" not in i]

    def create_comment(self, number, body):
        _s, d = self._call("POST", "/repos/%s/issues/%s/comments"
                           % (self.repo, number), {"body": body})
        return (d or {}).get("id")

    def delete_comment(self, comment_id):
        # "/repos/issues/comments/{id}" (no owner/repo) 404s. Verified against the
        # live API: the correct route answers 403 with documentation_url
        # .../issues/comments#delete-an-issue-comment, i.e. the route is right
        # and only the permission was missing.
        self._call("DELETE", "/repos/%s/issues/comments/%s"
                   % (self.repo, comment_id))

    def close_issue(self, number):
        self._call("PATCH", "/repos/%s/issues/%s" % (self.repo, number),
                   {"state": "closed"})

    def set_labels(self, number, remove=(), add=()):
        for label in remove:
            try:
                self._call("DELETE", "/repos/%s/issues/%s/labels/%s"
                           % (self.repo, number,
                              urllib.parse.quote(label)))
            except Exception:
                pass
        if add:
            self._call("POST", "/repos/%s/issues/%s/labels"
                       % (self.repo, number), {"labels": list(add)})


class FakeTransport(object):
    """For tests and --dry-run: the same interface, no network, no side effects."""

    def __init__(self, issues=(), owner="owner", login=None, repo=None):
        # `.repo` must exist here: the self-test reports "<repo> (owner <login>)"
        # and a missing attribute made it print a FAILED repository check, i.e.
        # it blamed the user's token for a gap in the test double.
        self.repo = repo or ("%s/example" % owner)
        self.issues = list(issues)
        self.owner = owner
        self.login = login or owner
        self.comments = {}          # number -> [body, ...]
        self.labels = {}            # number -> set()
        self.calls = []

    def whoami(self):
        return self.login

    def repo_owner(self):
        return self.owner

    def list_jobs(self):
        out = []
        for it in self.issues:
            if it.get("state", "open") != "open":
                continue
            if JOB_LABEL not in [l["name"] if isinstance(l, dict) else l
                                 for l in it.get("labels", [])]:
                continue
            n = it["number"]
            if DONE_LABEL in self.labels.get(n, set()):
                continue
            out.append(it)
        return out

    def ensure_labels(self):
        pass

    def comment(self, number, body):
        self.calls.append(("comment", number))
        self.comments.setdefault(number, []).append(body)

    def create_issue(self, title, body, labels=(JOB_LABEL,)):
        number = 1000 + len(self.comments) + len(self.issues)
        self.issues.append({"number": number, "body": body, "state": "open",
                            "user": {"login": self.login},
                            "labels": [{"name": l} for l in labels],
                            "title": title,
                            "updated_at": "2026-01-01T00:00:00Z"})
        return ("https://example.invalid/issues/%d" % number, number,
                list(labels))

    def add_labels(self, number, labels):
        self.calls.append(("add_labels", number, tuple(labels)))
        for it in self.issues:
            if it["number"] == number:
                for l in labels:
                    it["labels"].append({"name": l})

    def remove_label(self, number, label):
        self.calls.append(("remove_label", number, label))
        for it in self.issues:
            if it["number"] == number:
                it["labels"] = [l for l in it["labels"] if l["name"] != label]

    def create_label(self, name, color="ededed", description=""):
        self.calls.append(("create_label", name))

    def delete_label(self, name):
        self.calls.append(("delete_label", name))

    def list_issues(self, limit=5):
        return [i for i in self.issues if i.get("state", "open") == "open"][:limit]

    def create_comment(self, number, body):
        self.comments.setdefault(number, []).append(body)
        return 4242

    def delete_comment(self, comment_id):
        self.calls.append(("delete_comment", comment_id))

    def close_issue(self, number):
        self.calls.append(("close_issue", number))

    def set_labels(self, number, remove=(), add=()):
        self.calls.append(("labels", number, tuple(remove), tuple(add)))
        s = self.labels.setdefault(number, set())
        for r in remove:
            s.discard(r)
        for a in add:
            s.add(a)


# ---------------------------------------------------------------------------
# the agent
# ---------------------------------------------------------------------------
class Agent(object):
    def __init__(self, transport, allow_users, dry_run=False, log=None,
                 verbose=False, state_path=None, max_jobs=10, clock=time.time):
        self.t = transport
        self.allow_users = {u.lower() for u in allow_users if u}
        self.dry_run = dry_run
        self.log_path = log
        self.verbose = verbose
        self.state_path = state_path
        self.max_jobs = max(1, int(max_jobs))
        self.clock = clock
        self.seen = self._load_state()

    # -- state --------------------------------------------------------------
    def _load_state(self):
        if not self.state_path or not os.path.exists(self.state_path):
            return set()
        try:
            with open(self.state_path) as fh:
                return set(json.load(fh).get("done", []))
        except Exception:
            return set()

    def _save_state(self):
        if not self.state_path:
            return
        try:
            tmp = self.state_path + ".tmp"
            with open(tmp, "w") as fh:
                json.dump({"done": sorted(self.seen)}, fh)
            os.replace(tmp, self.state_path)
        except Exception:
            pass

    # -- logging ------------------------------------------------------------
    def log(self, msg):
        line = "%s %s" % (time.strftime("%Y-%m-%dT%H:%M:%S"), redact(msg))
        if self.verbose:
            print(line, flush=True)
        if not self.log_path:
            return
        try:
            with open(self.log_path, "a") as fh:
                fh.write(line + "\n")
        except Exception:
            pass

    # -- one pass -----------------------------------------------------------
    def poll_once(self):
        """Returns the number of jobs handled. Never raises."""
        try:
            jobs = self.t.list_jobs()
        except Exception as e:
            self.log("poll failed: %s" % redact(str(e)))
            return 0
        handled = 0
        for issue in (jobs or [])[:self.max_jobs]:
            try:
                if self.handle(issue):
                    handled += 1
            except Exception:
                self.log("handler crashed on #%s:\n%s"
                         % (issue.get("number"), redact(traceback.format_exc())))
        return handled

    def handle(self, issue):
        number = issue.get("number")
        author = ((issue.get("user") or {}).get("login") or "")
        key = "%s/%s" % (number, (issue.get("updated_at") or "")[:19])

        if str(number) in self.seen or key in self.seen:
            self.t.set_labels(number, remove=(JOB_LABEL,), add=(DONE_LABEL,))
            return False

        # ---- control 1: GitHub's own authentication of the author ----------
        if author.lower() not in self.allow_users:
            self.log("REFUSED #%s from %r (not in the allowlist %s)"
                     % (number, author, sorted(self.allow_users)))
            self.seen.add(str(number))
            self._save_state()
            if not self.dry_run:
                self.t.comment(number, self._refusal_comment(
                    author, number))
                self.t.set_labels(number, remove=(JOB_LABEL,),
                                  add=(REFUSED_LABEL,))
            return True

        job = parse_job(issue)
        self.log("job #%s action=%s params=%s by %s"
                 % (number, job.action, job.params, author))

        if self.dry_run:
            argv, err = (None, None)
            a = REGISTRY.get(job.action or "")
            if a and a.argv:
                argv, err = a.build(job.params)
            self.log("  dry-run: would run %s (kind=%s, hardware_ok=%s) %s"
                     % (argv if argv else a.handler and "handler" or "nothing",
                        a.kind if a else "-", job.hardware_ok, err or ""))
            return True

        res = execute(job, base_env=dict(os.environ))
        meta = {
            "host": "%s@%s" % (_who(), socket.gethostname()),
            "cmdline": redact(" ".join(res.argv) if res.argv else "(nothing ran)"),
            "footer": ("Asked as `/pi %s` by @%s. This agent has no shell and "
                       "no flight controls: it runs a fixed list of read-only "
                       "actions. Full output is in the agent log on the Pi."
                       % (job.action or "?", author)),
            "log_path": self.log_path or "(no --log given)",
        }
        body = render_comment(job, res, meta)
        self.log("  exit=%s took=%.1fs out=%d bytes%s"
                 % (res.rc, res.seconds, len(res.out or ""),
                    " REFUSED" if res.refused else ""))
        if res.out:
            self.log("  --- output ---\n%s\n  --- end ---" % redact(res.out[:8000]))
        try:
            self.t.comment(number, body)
        except Exception as e:
            self.log("could not comment on #%s: %s" % (number, redact(str(e))))
            return True                        # leave the label on; retry later
        self.seen.add(str(number))
        self._save_state()
        if res.refused:
            self.t.set_labels(number, remove=(JOB_LABEL,), add=(REFUSED_LABEL,))
        else:
            self.t.set_labels(number, remove=(JOB_LABEL,), add=(DONE_LABEL,))
        return True

    def _refusal_comment(self, author, number):
        return "\n".join([
            "### pi-agent: refused",
            "",
            "Issue #%s came from @%s. This agent only obeys commands from %s."
            % (number, author, ", ".join("@" + u for u in sorted(self.allow_users))),
            "",
            "This repository is public, so anyone can open an issue here. If the "
            "agent ran whatever a comment said, a stranger would own this "
            "aircraft's computer. That is the whole reason for the allowlist - "
            "not rudeness.",
            "",
            "If you are the operator you are already authenticated as yourself; "
            "re-open this from that account.",
        ])


def read_credentials(args, env=None):
    """(token, repo), both stripped.

    A token pasted into a file, or set by a systemd EnvironmentFile, very often
    carries a trailing newline. A newline inside an Authorization header is a
    401 that reads like a bad token, and it is the first thing that goes wrong
    when the token comes from a file rather than an interactive export. The repo
    name is stripped for the same reason - it goes into the URL.
    """
    env = os.environ if env is None else env
    token = (env.get(args.token_env) or "").strip()
    repo = (args.repo or env.get("PI_AGENT_REPO") or "").strip()
    return token, repo


def selftest(transport, log=print):
    """Check every permission the agent needs, and name the one that is missing.

    Written because the failure mode is genuinely confusing: a token can create
    an issue and create a label, and still be unable to label an issue or comment
    on one - and GitHub reports the resulting 403 as "Resource not accessible by
    integration", which says nothing about which permission to fix.

    Everything it creates it also removes. It never runs an action.

    Two lessons from the first version of this, both now pinned by tests:

    * A cleanup step that fails must NOT be reported as a missing permission.
      The first version decided what counted by substring-matching step names for
      "delete"/"close", so its own duplicate label removal - which 404s because
      the label was already gone - was counted as a required step, and the tool
      told a perfectly good token that its permissions were missing. Blaming the
      wrong thing is worse than saying nothing, so `required` is now explicit.
    * Removing something that is already gone is success, not failure.
    """
    results = []          # (required, ok, name, detail, hint)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    temp_label = "pi:selftest-%s" % stamp

    def _trim(text):
        text = "" if text is None else str(text)
        return text if len(text) <= 300 else text[:300] + " ...(truncated)"

    def step(name, fn, hint="", required=True):
        try:
            got = fn()
            results.append((required, True, name, _trim(got), hint))
            return got
        except Exception as e:
            results.append((required, False, name, redact(str(e)), hint))
            return None

    def cleanup(name, fn):
        """Already-gone counts as cleaned."""
        try:
            fn()
            results.append((False, True, name, "", ""))
        except Exception as e:
            msg = redact(str(e))
            gone = ("404" in msg or "Not Found" in msg or "does not exist" in msg
                    or "already" in msg.lower())
            results.append((False, gone, name, "already gone" if gone else msg,
                            "a cleanup problem, not a permission problem"))

    login = step("token is valid (GET /user)", transport.whoami)
    # getattr: a transport missing `.repo` must not be reported as a token
    # problem. A self-test that blames the wrong thing is worse than no test.
    step("repo is readable (GET /repos)",
         lambda: "%s (owner %s)" % (getattr(transport, "repo", "?"),
                                    transport.repo_owner()))
    created_label = bool(step("create a repo label (Issues: write)",
                              lambda: (transport.create_label(
                                  temp_label, "ededed",
                                  "pi_agent self-test, safe to delete"), True)[1]))

    # Borrow an open issue if there is one; otherwise make a throwaway.
    found = step("list open issues (Issues: read)",
                 lambda: transport.list_issues(5)) or []
    issue_no, created_issue = None, False
    if found:
        issue_no = found[0].get("number")
    else:
        issue_no = step("create an issue (Issues: write)",
                        lambda: transport.create_issue(
                            "pi_agent self-test - safe to close",
                            "/pi selftest-marker", ())[1])
        created_issue = bool(issue_no)

    labelled = commented = None
    if issue_no:
        labelled = bool(step("label an issue (the one that usually fails)",
                             lambda: (transport.add_labels(
                                 issue_no, [temp_label]), True)[1],
                             hint="needs Issues: Read and write, not just read"))
        if labelled:
            # This is the DELETE-permission check. Its cleanup twin used to run
            # again later and 404, which is what broke the verdict.
            step("remove that label again",
                 lambda: (transport.remove_label(issue_no, temp_label), True)[1])
        commented = step("comment on an issue (how results come back)",
                         lambda: transport.create_comment(
                             issue_no, "pi_agent self-test comment - safe to delete"))

    if commented:
        cleanup("delete the test comment", lambda: transport.delete_comment(commented))
    if created_label:
        cleanup("delete the test label from the repo",
                lambda: transport.delete_label(temp_label))
    if created_issue and issue_no:
        cleanup("close the throwaway issue",
                lambda: transport.close_issue(issue_no))

    missing = [r for r in results if r[0] and not r[1]]
    ok = not missing
    lines = ["pi_agent self-test", "=" * 60]
    for required, good, name, detail, hint in results:
        tag = "" if required else "  (cleanup)"
        lines.append("  %s %s%s" % ("PASS" if good else "FAIL", name, tag))
        if detail:
            lines.append("       %s" % detail)
        if not good and hint:
            lines.append("       -> %s" % hint)
    lines.append("=" * 60)
    if ok:
        lines.append("All required permissions are present. You are good to go.")
    else:
        lines.append("These permissions are MISSING, so the agent cannot work:")
        for _req, _good, name, _d, _h in missing:
            lines.append("  - %s" % name)
        lines.append("")
        lines.append("Fix the token: github.com/settings/tokens -> the token ->")
        lines.append("  Issues: Read and write   (NOT Read-only)")
        lines.append("A 'Read-only' Issues permission can still create issues and")
        lines.append("labels, which is exactly what makes this failure confusing.")
    if any(not r[0] and not r[1] for r in results):
        lines.append("")
        lines.append("(Some cleanup could not complete - that does not affect the")
        lines.append(" verdict above, but it may have left a test label or comment")
        lines.append(" behind.)")
    lines.append("")
    lines.append("Nothing was executed on this machine - this only talks to GitHub.")
    text = "\n".join(lines)
    if log:
        log(text)
    return ok, text


def queue_job(transport, command, title=None):
    """Validate a command and queue it. Returns (ok, message).

    The command is parsed and validated with the SAME code the poller uses, so a
    typo is refused here with the same message the Pi would have sent back - it
    never becomes a mystery issue sitting unlabelled in the repository.
    """
    body = command.strip()
    if not body.startswith("/pi"):
        return False, "a command starts with `/pi` (for example `/pi scout`)"
    job = parse_job({"number": 0, "body": body, "user": {"login": "local"}})
    if job.error:
        return False, job.error
    action = REGISTRY.get(job.action)
    err = action.check(job.params)
    if err:
        return False, err
    title = title or ("pi: %s" % (job.action +
                                  (" " + " ".join("%s=%s" % kv for kv in
                                                  sorted(job.params.items()))
                                   if job.params else "")))
    url, number, got = transport.create_issue(title, body)
    if JOB_LABEL not in got:
        return False, (
            "the issue was created (%s) but the `%s` label did NOT stick, so the "
            "agent will never pick it up. The token can create issues but not "
            "label them - it needs Issues: Read and write. Add the label by hand "
            "in the browser, or fix the token and re-queue." % (url, JOB_LABEL))
    return True, "queued %s as %s (#%s)" % (body, url, number)


def _who():
    try:
        return os.environ.get("USER") or os.environ.get("LOGNAME") or "pi"
    except Exception:
        return "pi"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Run a fixed, read-only list of actions on this Pi when "
                    "the repository owner asks for them in a GitHub issue.")
    ap.add_argument("--repo", default=None,
                    help="owner/name (default $PI_AGENT_REPO)")
    ap.add_argument("--token-env", default="PI_AGENT_TOKEN",
                    help="env var holding the GitHub token (default "
                         "%(default)s). The token needs issues:write and "
                         "metadata:read on this one repo, and nothing else.")
    ap.add_argument("--allow-user", action="append", default=[],
                    help="GitHub login allowed to give orders; repeatable. "
                         "Default: the repository owner only.")
    ap.add_argument("--once", action="store_true", help="one pass, then exit")
    ap.add_argument("--loop", action="store_true", help="poll until stopped")
    ap.add_argument("--interval", type=float, default=20.0,
                    help="seconds between polls (default %(default)s)")
    ap.add_argument("--dry-run", action="store_true",
                    help="show what would run; execute nothing, post nothing")
    ap.add_argument("--max-jobs", type=int, default=10,
                    help="most jobs to handle per pass (default %(default)s)")
    ap.add_argument("--state", default=os.path.expanduser("~/.pi_agent_state.json"),
                    help="remembered job ids, so a restart does not re-run them")
    ap.add_argument("--log", default=os.path.expanduser("~/.pi_agent.log"),
                    help="append-only audit log (default %(default)s)")
    ap.add_argument("--list-actions", action="store_true",
                    help="print the allowlist and exit")
    ap.add_argument("--selftest", action="store_true",
                    help="check that the token has every permission the agent "
                         "needs, then exit. Creates and removes its own test "
                         "label; runs no action")
    ap.add_argument("--queue", metavar="COMMAND",
                    help="validate a command and queue it as a labelled issue, "
                         "then exit (e.g. --queue '/pi scout')")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)

    if args.list_actions:
        print(help_text())
        return 0

    # Credentials FIRST: --repo's argparse default is None now, so resolving
    # $PI_AGENT_REPO has to happen before the check or the env var never works.
    token, args.repo = read_credentials(args)
    if not args.repo:
        print("need --repo owner/name or $PI_AGENT_REPO", file=sys.stderr)
        return 2
    fake = FakeTransport() if (args.dry_run and not token) else None
    if fake is not None:
        print("no token in $%s - dry-run will show the allowlist but no real "
              "issues" % args.token_env)

    try:
        transport = fake or GitHubTransport(args.repo, token,
                                            verbose=args.verbose)
        owner = transport.repo_owner() or ""
        allow = list(args.allow_user)
        if not allow:
            if not owner:
                print("could not determine the repository owner; pass "
                      "--allow-user", file=sys.stderr)
                return 2
            allow = [owner]
        transport.ensure_labels()
        me = ""
        try:
            me = transport.whoami() or ""
        except Exception:
            pass
        if me and me.lower() not in {a.lower() for a in allow}:
            print("WARNING: this token belongs to %r, which is not in the "
                  "allowlist %s - it will poll, but it will refuse its own "
                  "commands. That is correct." % (me, allow), file=sys.stderr)
    except Exception as e:
        print("startup failed: %s" % redact(str(e)), file=sys.stderr)
        return 2

    if args.selftest:
        ok, _text = selftest(transport)
        return 0 if ok else 1

    if args.queue:
        ok, msg = queue_job(transport, args.queue)
        print(("queued: " if ok else "not queued: ") + msg)
        return 0 if ok else 1

    agent = Agent(transport, allow, dry_run=args.dry_run, log=args.log,
                  verbose=args.verbose, state_path=args.state,
                  max_jobs=args.max_jobs)
    agent.log("started: repo=%s allow=%s dry_run=%s pid=%d"
              % (args.repo, allow, args.dry_run, os.getpid()))
    if args.dry_run:
        print(help_text())
        print()

    if not args.once and not args.loop:
        # Default to --once: a stray foreground loop on an aircraft computer is
        # worse than re-running one command.
        args.once = True

    try:
        while True:
            n = agent.poll_once()
            if n:
                agent.log("handled %d job(s)" % n)
            if args.once:
                break
            time.sleep(max(5.0, args.interval))
    except KeyboardInterrupt:
        agent.log("interrupted")
    except Exception:
        agent.log("fatal:\n%s" % redact(traceback.format_exc()))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
