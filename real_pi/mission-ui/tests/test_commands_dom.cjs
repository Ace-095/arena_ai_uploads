/* Real index.html + app.js DOM event handlers -> live Pi HTTP fixture.
 * Map drawing, transport displays and MP are stubs; Pi command requests are NOT.
 * Called by mission_pi/tests/test_ui_command_api.py (do not point at a real FC).
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');
const root = path.resolve(__dirname, '..');
const base = process.argv[2];
if (!base) throw new Error('Use the Python test fixture, not a live flight controller');
const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
  url: 'http://laptop.test', runScripts: 'outside-only', pretendToBeVisual: true,
});
const w = dom.window, $ = id => w.document.getElementById(id);
const calls = [], pending = new Set();
const vertices = [{lat:0,lon:0},{lat:0,lon:.001},{lat:.001,lon:.001},{lat:.001,lon:0}];
const noop = () => {};
const fakeMap = new Proxy({ getPolygonVertices: () => vertices, getVertices: () => vertices,
  polygonToFence: () => vertices }, {get: (obj,k) => obj[k] || noop});
w.MissionMap = {initMap: () => fakeMap};
w.MissionLinks = {
  initPiLink: opts => ({connect: () => opts.onStatus({connected:true}), close:noop, probe: async () => null}),
  initBridge: () => ({connect:noop}),
};
w.MissionCameras = {initCameras: () => new Proxy({}, {get: () => noop})};
w.confirm = () => true;
w.fetch = (url, opts={}) => {
  const u = new URL(url, w.location.href).href;
  calls.push({url:u, ...opts});
  let p;
  if (u.startsWith(base)) p = fetch(u, opts);
  else {
    const d = u.endsWith('/api/mp/state') ? {mock:false} : {};
    p = Promise.resolve(new Response(JSON.stringify(d), {status:200}));
  }
  pending.add(p);
  p.finally(() => pending.delete(p));
  return p;
};
const sleep = ms => new Promise(r => setTimeout(r,ms));
async function settle() {
  // Allow response.json + event-handler promise chains and camera debounce.
  for (let i=0; i<5; i++) { await Promise.allSettled([...pending]); await sleep(30); }
}
async function click(id) { assert.ok($(id), id); $(id).click(); await settle(); }
function writes() { return calls.filter(c => c.method && c.method !== 'GET' && !c.url.endsWith('/api/camera/status')); }
function lastBody() { return JSON.parse(writes().at(-1).body); }
async function fault(fail) {
  await fetch(base+'/__test/fault', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({fail})});
}
async function main() {
  w.eval(fs.readFileSync(path.join(root,'js/app.js'),'utf8'));
  await settle();
  $('piUrl').value = base;
  await click('btnPiConnect');
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
  assert.match($('pConverted').textContent, /cleared \(FC readback\)/);
  assert.equal($('fVerts').textContent, '0');
  await fault('upload');
  await click('btnFenceApply');
  assert.match($('pConverted').textContent, /NOT confirmed/);
  await fault(null);
  console.log('PASS DOM fence upload/polygon/refresh/clear, rejection, no false success, one writer');

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
  await sleep(600); await settle();
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
}
main().then(() => dom.window.close()).catch(e => {console.error(e);dom.window.close();process.exitCode=1;});
