/* Real index.html + app.js DOM event handlers -> live Pi HTTP fixture.
 * Map drawing, transport displays and MP are stubs; Pi command requests are NOT.
 * Called by mission_pi/tests/test_ui_command_api.py (do not point at a real FC).
 */
const assert = require('node:assert/strict');
const { createHarness } = require('./harness.cjs');

const base = process.argv[2];
if (!base) throw new Error('Use the Python test fixture, not a live flight controller');

async function main() {
  const h = createHarness(base);
  const { w, $, calls, vertices, boot, settle, click, writes, lastBody, fault } = h;
  await boot();

  $('piUrl').value = base;
  await click('btnPiConnect');
  assert.match($('fWriter').textContent, /DIRECT Pi/, 'writer badge must say the Pi owns the fence');

  // --- camera auto-detect is visible, and re-scannable, from the UI -------
  // The operator's question on real hardware is "which camera did it pick and
  // which one is the downward/bottom one?" — that must be readable here, not
  // only in a terminal. The panel is filled from the Pi's own /api/cameras.
  assert.match($('camDetected').textContent, /detected:/,
               'the panel must list what the Pi auto-detected');
  assert.match($('camDetected').textContent, /cam1 \(front\)/);
  assert.match($('camDetected').textContent, /cam2 \(bottom\)/);
  assert.match($('tabCam2').textContent, /^CAM2 · /,
               'tab labels follow the hardware, not the datasheet');
  calls.length = 0;
  await click('btnCamScan');
  assert.equal(calls.filter(c => c.url.endsWith('/api/cameras/rescan')).length, 1,
               'scan cams must ask the Pi to re-detect');
  assert.equal(calls.filter(c => c.url.endsWith('/api/cameras')).length >= 1, true,
               'after a scan the panel must be re-read');
  assert.match($('camDetected').textContent, /detected:/,
               'the detected list must survive a rescan');
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(writes().length, 1, 'ONE fence writer, not Pi + MP');
  assert.equal(writes()[0].url, base+'/api/fence');
  assert.deepEqual(lastBody().vertices, vertices);
  assert.match($('fReadback').textContent, /MATCH/);
  calls.length = 0;
  await click('btnPolyApply');
  assert.equal(writes().length, 1);
  assert.equal(writes()[0].url, base+'/api/fence');
  calls.length = 0;
  await click('btnFenceRefresh');
  assert.equal(calls.length, 1);
  assert.equal(calls[0].url, base+'/api/fence');
  await fault('clear');
  await click('btnFenceClear');
  assert.match($('pConverted').textContent, /NOT confirmed/);
  assert.notEqual($('fState').textContent, 'LOADED on FC');
  assert.equal($('btnFenceClear').disabled, false);
  await fault(null);
  await click('btnFenceClear');
  assert.match($('pConverted').textContent, /cleared \(FC readback/);
  assert.equal($('fVerts').textContent, '0');
  await fault('upload');
  await click('btnFenceApply');
  assert.match($('pConverted').textContent, /NOT confirmed/);
  await fault(null);
  console.log('PASS DOM fence upload/polygon/refresh/clear, rejection, no false success, one writer');

  // --- the writer must be PROVEN, never assumed -------------------------
  // A URL that is not a verified Pi may not receive a fence write. This is
  // the bug that made "UPLOAD FENCE" 404 against the laptop bridge.
  $('piUrl').value = 'http://not-the-pi.test:8000';
  await click('btnPiConnect');
  assert.match($('fWriter').textContent, /BLOCKED/);
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(writes().length, 0, 'unverified host must not receive a fence write');
  assert.equal(calls.filter(c => c.url.includes('/api/fence')).length, 0);
  assert.match($('pConverted').textContent, /REFUSED/);
  assert.match($('pConverted').textContent, /not mission_pi|bridge|Pi URL/);

  // ...unless the operator deliberately opts into the MP relay
  $('chkMpFenceFallback').checked = true;
  $('chkMpFenceFallback').dispatchEvent(new w.Event('change', {bubbles:true}));
  assert.match($('fWriter').textContent, /Mission Planner/);
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(writes().at(-1).url, 'http://laptop.test/api/mp/fence',
               'explicit fallback goes to the MP bridge, and says so');
  assert.match($('fWriter').textContent, /not direct/i);
  $('chkMpFenceFallback').checked = false;
  $('chkMpFenceFallback').dispatchEvent(new w.Event('change', {bubbles:true}));

  // back to the direct path: the Pi is the only writer again
  $('piUrl').value = base;
  await click('btnPiConnect');
  assert.match($('fWriter').textContent, /DIRECT Pi/);
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(writes().length, 1);
  assert.equal(writes()[0].url, base+'/api/fence');
  console.log('PASS DOM fence writer proven before use, blocked without a Pi, MP only on explicit opt-in');

  await click('tabCam1');
  calls.length = 0;
  await click('btnQrBoostDay');
  assert.equal(writes().at(-1).url, base+'/api/camera/controls');
  assert.equal(lastBody().cam, 'cam1');
  assert.equal(lastBody().qr_boost_profile, 'qr_boost_day');
  await click('tabCam2');
  await click('btnQrBoostAgg');
  assert.equal(lastBody().cam, 'cam2');
  let st = await (await fetch(base+'/api/camera/status?cam=cam2')).json();
  assert.equal(st.controls.contrast, 2);
  await click('btnQrBoostOff');
  st = await (await fetch(base+'/api/camera/status?cam=cam2')).json();
  assert.equal(st.controls.qr_boost_enabled, false);
  $('qrSwMode').value = 'none';
  await click('btnQrSwApply');
  st = await (await fetch(base+'/api/camera/status?cam=cam2')).json();
  assert.equal(st.controls.qr_software_enhance, false);
  $('c1Exp').value = '7000';
  $('c1Exp').dispatchEvent(new w.Event('input', {bubbles:true}));
  await new Promise(r => setTimeout(r, 600)); await settle();
  st = await (await fetch(base+'/api/camera/status?cam=cam1')).json();
  assert.equal(st.controls.exposure_us, 7000);
  $('qrPreset').value = 'dark';
  await click('btnQrPresetApply');
  st = await (await fetch(base+'/api/qr/presets')).json();
  assert.equal(st.active, 'dark');
  console.log('PASS DOM camera selection/boost/off/software/slider/preset -> live Pi state');

  $('cfgMaxInput').value = '12'; $('cfgSweepInput').value = '10';
  await click('btnCfgApply');
  st = await (await fetch(base+'/api/config')).json();
  assert.equal(st.max_alt_m, 12); assert.equal(st.sweep_alt_m, 10);
  await fault('param'); await click('btnCfgApply');
  assert.match($('logBox').textContent, /NOT verified: no parameter echo/);
  await fault(null);
  console.log('PASS DOM altitude -> Pi, failed FC echo visibly warned');

  calls.length = 0;
  await click('btnAbort');
  assert.equal(writes().at(-1).url, 'http://laptop.test/api/mp/mode');
  assert.equal(lastBody().mode, 'RTL');
  await click('btnLand'); assert.equal(lastBody().mode, 'LAND');
  console.log('PASS DOM RTL/LAND intentional bridge routing (MP stub; not hardware tested)');

  h.dom.window.close();
}

main().then(() => process.exit(0)).catch(e => { console.error(e); process.exit(1); });
