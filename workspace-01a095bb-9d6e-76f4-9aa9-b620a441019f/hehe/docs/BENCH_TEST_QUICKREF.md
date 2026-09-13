# Bench Test Quick Reference — Pi + Pixhawk + Telemetry + Mission Planner

No propellers. Print this or keep it open on your phone.

---

## Power-up order
1. [ ] Pixhawk + GPS first, outdoors/near a window — wait for GPS lock
2. [ ] Pi 5 second

## Physical connections
- [ ] Pixhawk TELEM1 → telemetry radio (air unit)
- [ ] Ground telemetry radio → laptop USB
- [ ] Pixhawk main USB → Pi 5 USB
- [ ] GPS/compass → Pixhawk GPS port
- [ ] Pi 5 → Wi-Fi router
- [ ] Laptop → same Wi-Fi router (for the browser)

---

## SITL rehearsal (optional, do this first)

```bash
# Terminal 1
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14552

# Terminal 2
cd backend
export MAVLINK_DEVICE=udpout:127.0.0.1:14552
uvicorn app:app --host 0.0.0.0 --port 8000
```
Mission Planner → connect UDP `127.0.0.1:14550`. Browser → `http://localhost:8000`.

---

## Real hardware

### On the Pi
```bash
ls /dev/serial/by-id/
export MAVLINK_DEVICE=/dev/serial/by-id/<your-id>
cd backend && source .venv/bin/activate
uvicorn app:app --host 0.0.0.0 --port 8000
```
```bash
hostname -I        # get the Pi's IP for the browser
```

### On the laptop
- Mission Planner → connect via ground telemetry radio's COM port (baud: check your radio's manual)
- Browser → open `frontend/index.html`, backend URL = `http://<pi-ip>:8000`

---

## Test checklist

- [ ] **1.** `curl http://<pi-ip>:8000/api/health` → ok
- [ ] **2.** Browser telemetry: live position, real GPS sat count
- [ ] **3.** System Info panel: real CPU/RAM/temp values
- [ ] **4.** Draw a small fence near your real location, upload
- [ ] **5.** Status: **ARMABLE** (green), `fc_readback_matched: true`
- [ ] **6.** ⭐ Check Mission Planner's fence display — auto-refreshed, or needs manual reload?
- [ ] **7.** ⭐ Read `FENCE_TYPE` param before/after upload — auto-inferred, or needed the bit set?
- [ ] **8.** `DELETE` the fence via browser → confirm it clears in Mission Planner too

⭐ = the two specific unknowns this test is meant to resolve — write down what you actually observe for both.

---

## If something's wrong

| Symptom | Check |
|---|---|
| No heartbeat / backend can't connect | Wrong `/dev/serial/by-id/` path, or Pixhawk not powered first |
| GPS sat count = 0 forever | Not enough sky view — move outdoors, wait longer |
| `fc_readback_matched: false` | Real bug — save the browser's `reason` field and the backend log |
| Browser can't reach Pi | Confirm both on the *same* Wi-Fi network, not just same router brand |
| Mission Planner shows no telemetry | Wrong COM port or baud rate for the radio |
