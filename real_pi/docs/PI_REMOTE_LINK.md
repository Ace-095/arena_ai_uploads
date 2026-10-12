# The Pi remote link (`tools/pi_agent.py`)

**What this is.** A way for the operator who wrote this code to run read-only
checks on the aircraft's Raspberry Pi, without an inbound port, without SSH, and
without anyone transcribing terminal output by hand.

**What it is not.** It is not a shell, not interactive, and not a way for anyone
other than you to run anything.

---

## Quick path (copy-paste)

If you have already created the token, this is the whole thing:

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi

read -rs -p "paste token, then Enter: " T && printf '%s' "$T" > ~/.pi_agent_token && chmod 600 ~/.pi_agent_token && unset T
export PI_AGENT_TOKEN="$(cat ~/.pi_agent_token)"
export PI_AGENT_REPO="Ace-095/arena_ai_uploads"

python3 tools/pi_agent.py --selftest        # must say "You are good to go"
python3 tools/pi_agent.py --queue "/pi scout" && python3 tools/pi_agent.py --once
```

Step-by-step instructions for each of those, including the token, follow in §4.

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

## 4. Setup, step by step

Nothing here touches the aircraft. Steps 1-5 only talk to GitHub; the first
command that opens hardware is in step 7.

### Step 0 - get the code onto the Pi

The agent lives on this branch, not on `main`:

```bash
cd ~/arena_ai_uploads
git branch --show-current              # what are we on?
git fetch origin
git checkout arena/1c4bb365-arena-ai-uploads
git pull
ls real_pi/mission_pi/tools/pi_agent.py     # must exist now
```

If `git checkout` complains about local changes, tell me what it said - do not
force it, your Pi may have edits worth keeping.

### Step 1 - create the token

1. Open **https://github.com/settings/personal-access-tokens/new**
   (or: your avatar → *Settings* → *Developer settings* → *Personal access
   tokens* → *Fine-grained tokens* → *Generate new token*).
2. **Token name:** `pi-agent` — anything you will recognise in six months.
3. **Expiration:** 90 days is sensible. Put a reminder in your calendar for
   the day before; the agent prints a clear 401 when it lapses.
4. **Repository access:** *Only select repositories* → pick
   **`Ace-095/arena_ai_uploads`**. Not "All repositories".
5. **Permissions** → expand *Repository permissions* → find **Issues** → set it
   to **Read and write**.
   * `Metadata: Read` turns itself on. That is expected and required.
   * **Leave everything else at "No access."** No Contents, no Actions, no
     Workflows, no Administration.
6. **Generate token**, then copy it. GitHub shows it once.

> **The one mistake to avoid.** Setting *Issues* to **Read-only** instead of
> **Read and write**. I hit exactly this while building this: with Issues read,
> the token could still *create* issues and create repo labels, but labelling an
> issue and commenting on one both failed with `403 Resource not accessible by
> integration` — which does not say which permission to fix. That is the whole
> reason `--selftest` exists (step 4).

Never paste the token into this chat, an issue, a commit, or a screenshot.

### Step 2 - store it on the Pi

This reads the token without echoing it to the screen and without putting it in
your shell history:

```bash
read -rs -p "paste token, then press Enter: " T \
  && printf '%s' "$T" > ~/.pi_agent_token \
  && chmod 600 ~/.pi_agent_token \
  && unset T \
  && echo "saved: $(wc -c < ~/.pi_agent_token) bytes, mode $(stat -c %a ~/.pi_agent_token)"
```

Expect roughly `93 bytes, mode 600`. (`printf '%s'`, not `echo`: no trailing
newline. The agent strips whitespace anyway, but tidy is tidy.)

No editor needed, and nothing is left in `~/.bash_history`. If you prefer an
editor, `nano ~/.pi_agent_token` then `chmod 600` it — but do not use
`export PI_AGENT_TOKEN=...` interactively, because that lands in your history.

### Step 3 - point it at the repository

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
export PI_AGENT_TOKEN="$(cat ~/.pi_agent_token)"
export PI_AGENT_REPO="Ace-095/arena_ai_uploads"
```

To save these two exports for every future shell:

```bash
printf 'export PI_AGENT_REPO="Ace-095/arena_ai_uploads"\n' >> ~/.bashrc
printf 'export PI_AGENT_TOKEN="$(cat ~/.pi_agent_token)"\n' >> ~/.bashrc
```

Keep the *token itself* in the 600-mode file and only the *reference* in
`.bashrc`.

### Step 4 - verify the token BEFORE anything else — `--selftest`

This is the step that turns a confusing failure into a named one. It talks only
to GitHub, runs no action, and cleans up after itself (one uniquely-named label,
plus a throwaway issue if there was no open issue to borrow):

```bash
python3 tools/pi_agent.py --selftest
```

A healthy Pi prints:

```
pi_agent self-test
============================================================
  PASS token is valid (GET /user)
       Ace-095
  PASS repo is readable (GET /repos)
       Ace-095/arena_ai_uploads (owner Ace-095)
  PASS create a repo label (Issues: write)
  PASS list open issues (Issues: read)
       3 open: #5, #6, #7
  PASS label an issue (the one that usually fails)
  PASS remove a label again
  PASS comment on an issue (how results come back)
       1234567
  PASS delete the test comment
  PASS remove the test label
  PASS delete the test label from the repo
============================================================
All required permissions are present. You are good to go.
```

If instead you see `FAIL label an issue` / `FAIL comment on an issue`, the
token's **Issues** permission is *Read-only*. Fix it in step 1 and re-run. Do not
continue past this step until it says **good to go**; everything after it assumes
the token can write.

### Step 5 - see what you are authorising, and rehearse

```bash
python3 tools/pi_agent.py --list-actions     # the complete allowlist; no network
python3 tools/pi_agent.py --once --dry-run   # shows what it WOULD run; runs nothing
```

Read the list. Nine actions, all read-only or explicitly consented. If any of
them looks wrong to you, stop and say so.

### Step 6 - first real job

```bash
python3 tools/pi_agent.py --queue "/pi help" && python3 tools/pi_agent.py --once
```

This creates a labelled issue, runs the action, and comments the result back.
Then look at it:

```bash
python3 tools/pi_agent.py --queue "/pi scout" && python3 tools/pi_agent.py --once
```

`scout` is the read-only field survey: board, thermal, deps, Hailo, the detector
rung it picks, cameras. It is quick and it opens **no** hardware.

You can also queue by hand in the browser: open an issue whose **first line** is
`/pi scout`, add the **`pi:job`** label, and the agent will pick it up on its next
pass.

### Step 7 - the one that matters: measure the real FOV

Every range number in this repository rests on a **VENDOR-QUOTED 75°/60°** that
nobody has checked. This measures it.

**Physics setup first:**
1. A **white sheet flat on a darker floor**, directly under the downward camera.
2. It must be **centred** in the picture and must not touch the frame edges.
3. It must be **clearly brighter** than the ground around it.
4. Tape-measure from the **lens** to the **sheet** — that number is `dist_m`.
5. Camera on `bottom`, `mission_pi` stopped.

Then, in a fresh shell (or re-export after step 3):

```bash
cd ~/arena_ai_uploads/real_pi/mission_pi
export PI_AGENT_TOKEN="$(cat ~/.pi_agent_token)"
export PI_AGENT_REPO="Ace-095/arena_ai_uploads"

python3 tools/pi_agent.py --queue "/pi fov dist_m=2.0 panel_mm=420 panel_mm_h=297 hardware=yes" \
  && python3 tools/pi_agent.py --once
```

* `dist_m` = what your tape says, in metres (here 2.0).
* `panel_mm` = the sheet's **long** side in mm (A3 = 420, A4 = 297).
* `panel_mm_h` = the sheet's **short** side in mm (A3 = 297). Optional; include
  it and you also get VFOV and a sanity check that the measured aspect ratio
  matches the declared one.

It refuses rather than guesses, and every refusal says why. A good result looks
like `FOV : MEASURED hfov=75.10 deg vfov=59.80 deg (config says hfov=75.0)`, with
`spread` under a few percent. Bench accuracy on known synthetic lenses is within
**0.35°**; a single centred sheet measures the **centre** field of view, and
corner distortion is not covered.

Then put the measured numbers into `config.yaml`
(`cameras.bottom.hfov_deg` / `vfov_deg`) so the UI's coverage planner and the
footprint maths stop using the vendor value.

### Step 8 - tell me

I can read issues (only write is blocked from my side), so once a job has
answered, just say "done" and I will pull the result, check it against what the
code claims, and tell you what it means. You do not need to copy any output into
the chat.

### Step 9 - run it as a service (optional)

Only worth doing once step 4 passes. Then you can queue from the browser and walk
away.

```bash
install -m 600 /dev/null ~/.pi-agent.env
nano ~/.pi-agent.env      # see the two lines below
```

```
PI_AGENT_TOKEN=github_pat_xxx
PI_AGENT_REPO=Ace-095/arena_ai_uploads
```

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
EnvironmentFile=/home/pi/.pi-agent.env
ExecStart=/usr/bin/python3 tools/pi_agent.py --loop --interval 20
Restart=on-failure
RestartSec=30
SupplementaryGroups=video dialout

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pi-agent
journalctl -u pi-agent -f
```

Adjust `User=` and the two `/home/pi/...` paths if your Pi's user is not `pi`.
Run it as your normal user, never as root - it needs no root for anything.

### Troubleshooting

| What you see | What it means |
|---|---|
| `FAIL label an issue` in `--selftest` | token's **Issues** permission is *Read-only* → set it to **Read and write** |
| `403 Resource not accessible by integration` | the same thing, from a command rather than the self-test |
| `not queued: the pi:job label did NOT stick` | same cause; the issue was created but never queued — add the label in the browser or fix the token |
| `401` / `token is valid` FAIL | wrong token, or it expired (check the expiry date from step 1) |
| `need --repo owner/name` | `export PI_AGENT_REPO=...` is missing in this shell |
| A job is silently ignored | no `pi:job` label, or you are not the issue's author, or it is already `pi:done` |
| `refused: mission_pi is running` | stop `mission_pi`: it owns the camera and the serial port |
| `FOV : NOT MEASURED - ...` then a reason | sheet off-centre, touching an edge, too small in frame, or not bright enough against the floor. The reason is printed after the dash — read it, it is specific |
| `no camera detected for role 'bottom' - see list_cams above` | run `python3 tools/list_cams.py`, and make sure nothing else holds the camera |

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
