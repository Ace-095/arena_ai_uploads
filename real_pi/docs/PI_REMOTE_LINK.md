# The Pi remote link (`tools/pi_agent.py`)

**What this is.** A way for the operator who wrote this code to run read-only
checks on the aircraft's Raspberry Pi, without an inbound port, without SSH, and
without anyone transcribing terminal output by hand.

**What it is not.** It is not a shell, not interactive, and not a way for anyone
other than you to run anything.

---

## 1. Why it works this way

The machine that wrote this repo has no route to your LAN. That is measured, not
assumed:

| Destination | Result | Reading |
|---|---|---|
| `api.github.com`, `github.com`, `codeload.github.com` | **200** | reachable |
| `registry.npmjs.org`, `pypi.org` | **200** | reachable |
| `example.com`, `webhook.site`, `ngrok.com` | `SSL_ERROR_SYSCALL` | blocked at TLS |
| `http://192.168.1.50` | timeout after 6 s | no route to a private address |

So: nothing can connect *in* to your Pi, and no tunnel service is reachable
either. **GitHub is the only channel we share** — and your Pi can reach it,
because your Pi can reach the internet normally.

### Why not a self-hosted GitHub Actions runner

That is the established way to run jobs on your own hardware, and it is the wrong
tool here. GitHub's own documentation: *"We recommend that you only use
self-hosted runners with private repositories. This is because forks of your
public repository can potentially run dangerous code on your self-hosted runner
machine by creating a pull request that executes the code in a workflow."*
([docs.github.com](https://docs.github.com/en/actions/how-tos/manage-runners/self-hosted-runners/add-runners);
see also [Wiz's hardening guide](https://www.wiz.io/blog/github-actions-security-guide),
which is blunter: *"Self-hosted runners should not be used with public
repositories"*.)

`Ace-095/arena_ai_uploads` is **public**, so a runner would be a permanent,
documented remote-code-execution path onto the Pi. A polling agent that can only
execute nine fixed read-only actions is a much smaller thing to get wrong.

The polling pattern itself is standard practice — e.g.
[relay.remotify.run](https://github.com/onlsol/relay.remotify.run) ("push commands
from an LLM, CI runner, or any HTTP client to a machine with only curl + bash...
No SSH port, agent, or inbound access required") and
[remote_pi](https://github.com/jacobaraujo7/remote_pi/blob/main/relay/README.md),
which authenticates devices with Ed25519 challenge-response. Both make the same
point: **the transport is easy; authenticating the sender is the whole job.**

---

## 2. How it works

```
   operator (me)            GitHub (the only shared host)              your Pi
        |                              |                                  |
        |-- read replies (GET) ------->|                                  |
        |                              |<---- poll every 20 s ------------|
        |   I cannot write here        |      GET issues?labels=pi:job     |
        |   (see §5)                   |      (outbound HTTPS only)        |
        |                              |                                   |
        |                              |<---- POST the result as a comment |
        |                              |<---- POST label pi:done ----------|
```

1. A command is queued as an **issue whose first line is `/pi <action>`**, carrying
   the **`pi:job`** label.
2. The agent polls for open issues with that label, newest last, and for each one:
   checks the author, validates the command, runs it, and posts the output as a
   comment.
3. The issue is relabelled `pi:done` (or `pi:refused`), so **a job never runs
   twice** — not after a restart either, because the issue number is also written
   to the state file.

The label is the queue, the comment is the answer, and the issue is the audit
trail. You can watch the whole exchange in the GitHub UI.

---

## 3. The security model

Your repository is public, so **anyone on the internet can open an issue**. Three
controls stand between a stranger and your aircraft's computer. All three have to
fail at once.

| # | Control | What it stops |
|---|---|---|
| 1 | **Author allowlist** — a job runs only if the issue author's login is in the allowlist (default: the repository owner). Read from the API's `user.login`, which is server-side truth. | A stranger's issue. Nothing in the body can forge the author. |
| 2 | **No shell.** The body selects an action from a fixed registry and supplies *typed* parameters. There is no interpreter, and no free-form string ever reaches a command line. | `; rm -rf /`, `$(reboot)`, `\| tee /etc/passwd` — none of it is escaped, it simply is not a valid parameter. |
| 3 | **Nothing actuates.** No arming, no mode change, no parameter write, no config write, no fence upload. A test fails the suite if such an action is ever added. | The agent cannot fly, disarm, or reconfigure anything, even if fully compromised. |

Anything that opens the **camera** or the **flight-controller serial port** is
classed `hardware` and additionally requires:

* an explicit `hardware=yes` in the command, and
* `mission_pi` **not** running — it holds those devices, and two owners on one CSI
  camera or one serial port is a support call, not a measurement.

The agent is otherwise read-only by construction. Every one of those controls is
pinned by a test, and each was verified by mutating it and confirming the suite
fails (11 mutations, all caught).

### If a stranger does open an issue

They get a comment explaining that they are not the operator, the label
`pi:refused`, and nothing runs. The refusal names the operator's login but no
token, no path, and no environment variable.

---

## 4. Setup

### 4.1 Create a token for the Pi

GitHub → Settings → Developer settings → **Fine-grained personal access tokens**
→ Generate new token:

* **Repository access:** Only select repositories → `Ace-095/arena_ai_uploads`
* **Permissions:**
  * **Issues: Read and write** ← required
  * **Metadata: Read** ← comes with the above automatically
* Nothing else. Not `Contents`, not `Actions`, not `Workflows`.

**Why "Read and write" on Issues matters, measured:** a token that can create an
issue but not label or comment on one makes GitHub **create the issue and silently
drop the label** — no error, no timeline event, and the job is never picked up.
That is the worst possible outcome, so `--queue` re-reads the issue afterwards and
refuses to report success if the label did not stick.

Put the token in a file only you can read, not in your shell history:

```bash
install -m 600 /dev/null ~/.pi_agent_token
printf '%s' 'github_pat_...' > ~/.pi_agent_token   # paste it in your editor instead
chmod 600 ~/.pi_agent_token
```

### 4.2 First run (does nothing but look)

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
export PI_AGENT_TOKEN="$(cat ~/.pi_agent_token)"
export PI_AGENT_REPO="Ace-095/arena_ai_uploads"

python3 tools/pi_agent.py --list-actions     # what you are authorising
python3 tools/pi_agent.py --once --dry-run   # shows what it WOULD do; runs nothing
```

### 4.3 Queue a job and answer it

```bash
# queue it (creates the labelled issue) and immediately handle the queue
python3 tools/pi_agent.py --queue "/pi scout" && python3 tools/pi_agent.py --once
```

Or queue several and let the loop drain them:

```bash
python3 tools/pi_agent.py --queue "/pi hailo"
python3 tools/pi_agent.py --queue "/pi capture role=bottom hardware=yes"
python3 tools/pi_agent.py --once
```

You can also queue from the GitHub **web UI**: open an issue whose first line is
`/pi scout`, add the `pi:job` label. No CLI needed.

### 4.4 Run it as a service (optional)

```ini
# /etc/systemd/system/pi-agent.service
[Unit]
Description=pi_agent - GitHub-issue command relay
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/arena_ai_uploads/real_pi/mission_pi
EnvironmentFile=/home/pi/.pi-agent.env      # PI_AGENT_TOKEN=..., PI_AGENT_REPO=...
ExecStart=/usr/bin/python3 tools/pi_agent.py --loop --interval 20
Restart=on-failure
RestartSec=30
# it needs the camera and the serial port only when you ask for them
SupplementaryGroups=video dialout

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now pi-agent
journalctl -u pi-agent -f
```

Run it as your normal user, never as root. It does not need root for anything.

---

## 5. The action allowlist

This is the entire surface. Anything not on it is refused.

| Action | Kind | What it does |
|---|---|---|
| `help` | read-only | print this list |
| `scout` | read-only | `pi_field_report.py --no-capture` — the field survey |
| `cameras` | read-only | enumerate cameras and their roles |
| `hailo` | read-only | Hailo platform, devices, compiled model inventory |
| `tests` | read-only | the whole suite, on the Pi (slow: up to 45 min) |
| `fov` | hardware | **measure the real FOV** from a sheet of known size |
| `fence` | hardware | read the Pixhawk fence and compare it to the export |
| `verify` | hardware | the survey **plus** the Pixhawk |
| `capture` | hardware | one frame, attached as a downscaled JPEG |

```bash
/pi fov dist_m=2.0 panel_mm=420 hardware=yes
/pi fov dist_m=1.5 panel_mm=297 panel_mm_h=210 hardware=yes role=front
/pi capture role=bottom hardware=yes
/pi verify hardware=yes
```

Parameters are typed: numbers must be numbers and be in range, `role` is one of
`bottom|front`, and anything else is refused. **A new action means new code and a
new test** — there is deliberately no way to run an arbitrary command through
this.

`capture` downscales the frame until it fits in a comment (it tries 900→320 px and
quality 80→35) and posts it as base64, so the operator can actually look at what
the camera sees rather than reading a description of it.

---

## 6. Honest status: what is proven, and what is not

Written the same day it was built, so nobody has to guess later.

| Claim | Evidence | Status |
|---|---|---|
| The agent polls the real GitHub API and reads real issues | ran against `api.github.com`; read issues and labels | **verified** |
| The agent's write path works against the real API | it created the three `pi:` labels in the live repository (they carry its description) | **verified** |
| Commands are refused correctly | 64 tests, incl. strangers, shell syntax, paths, unknown actions, out-of-range numbers | **verified** |
| Each security control is real | 11 mutations (allowlist, consent gate, device arbitration, validation, killpg, redaction, truncation, idempotency, the label-drop guard, an added arming action) — every one made the suite fail | **verified** |
| **Posting the reply back to an issue** | **not exercised against the live API.** The sandbox's own GitHub credential is a GitHub App installation token without issue write: every attempt returns `403 Resource not accessible by integration`. The `documentation_url` in those errors resolves to the exact endpoints (`rest/issues/comments#create-an-issue-comment`), so the route is right and it is purely a permission refusal. The code path is covered by the test suite against a fake transport. | **UNVERIFIED — first thing your token will exercise** |
| Anything measured on your hardware | nothing in this document is a measurement of your rig | **NOT YET MEASURED** |

The honest summary: **the read half of this loop is proven end-to-end, the write
half is one permission away and has never run for real.** Expect the first `--once`
to be the moment that changes; if it fails it will say so in the log rather than
silently, because a comment failure deliberately does *not* mark the job done.

---

## 7. Operating notes

* **Latency** = the poll interval (default 20 s) + however long the job takes.
  Polling is cheap: conditional requests returning `304` are not charged against
  the GitHub rate limit.
* **One command per issue.** A processed issue is labelled `pi:done` and never
  re-runs. To run something again, open a new issue (or remove `pi:done` and add
  `pi:job` back).
* **Timeouts.** Every action has one (`fov` 300 s, `tests` 2700 s, ...). On
  timeout the **whole process group** is killed, because `run_tests.sh` spawns a
  tree and killing only the shell would leave a test run alive holding the camera.
* **The log** is `~/.pi_agent.log` — append-only, one line per job, output
  included. It is redacted: tokens are scrubbed before anything is written or
  posted.
* **Emergency stop.** `sudo systemctl stop pi-agent`, or `Ctrl-C` if it is in the
  foreground. Nothing it does outlives it.

### Troubleshooting

| Symptom | Cause |
|---|---|
| `not queued: the pi:job label did NOT stick` | token lacks **Issues: Read and write** — see §4.1 |
| `403 Resource not accessible by integration` on commenting | same: the token needs Issues write, not just read |
| A job is ignored | no `pi:job` label, or the author is not in the allowlist, or it is already `pi:done` |
| `refused: mission_pi is running` | stop `mission_pi` — it owns the camera and serial port |
| Nothing happens at all | check `PI_AGENT_TOKEN`/`PI_AGENT_REPO`, then `journalctl -u pi-agent` |
| It refuses its own commands | working as designed: the token belongs to a login that is not the operator |
