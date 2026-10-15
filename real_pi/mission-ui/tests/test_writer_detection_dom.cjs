/* Which server serves this page decides who may write the fence.
 *
 * This is the regression test for the real failure: the bridge serves the same
 * UI from the laptop, app.js assumed the origin was the Pi (S.piUrl =
 * location.origin), and "UPLOAD FENCE -> FC" POSTed to <laptop>/api/fence,
 * which the bridge does not implement. The operator saw a 404 and concluded
 * the UI cannot set a geofence.
 *
 * No Python fixture needed: /health is stubbed for each scenario, because the
 * question is only "what does the UI do with what the origin claims to be".
 * Run: node tests/test_writer_detection_dom.cjs
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');
const root = path.resolve(__dirname, '..');
const APP = fs.readFileSync(path.join(root, 'js/app.js'), 'utf8');
const HTML = fs.readFileSync(path.join(root, 'index.html'), 'utf8');

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const noop = () => {};

/* Serve the page from `origin`, answer /health with `health`, and record every
 * fetch the app makes. The Pi link reports `piLinked` and only verifies when
 * the app connects to a URL that really is the Pi under test. */
async function boot({ origin, health, piUrlIsPi = () => false, query = '' }) {
  const dom = new JSDOM(HTML, {
    url: origin + '/' + query, runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window, $ = (id) => w.document.getElementById(id);
  const calls = [];
  const vertices = [{lat:0,lon:0},{lat:0,lon:.001},{lat:.001,lon:.001},{lat:.001,lon:0}];
  const fakeMap = new Proxy({
    getPolygonVertices: () => vertices, getVertices: () => vertices,
    polygonToFence: () => vertices,
  }, {get: (o, k) => o[k] || noop});
  w.MissionMap = {initMap: () => fakeMap};
  let connectedUrl = null;
  w.MissionLinks = {
    initPiLink: (o) => ({
      connect: (u) => { connectedUrl = u; o.onStatus({connected: !!piUrlIsPi(u), url: u, retry: 0}); },
      close: noop,
      // mirrors links.js: the verdict arrives via onProbe
      probe: async () => {
        const res = piUrlIsPi(connectedUrl)
          ? {ok:true, isPi:true, ms:2, health}
          : {ok:false, ms:4000, error:'unreachable'};
        o.onProbe && o.onProbe(res);
        return res;
      },
    }),
    initBridge: () => ({connect:noop}),
  };
  w.MissionCameras = {initCameras: () => new Proxy({}, {get: () => noop})};
  w.confirm = () => true;
  w.fetch = (url, opts = {}) => {
    const u = new URL(url, w.location.href).href;
    calls.push({url: u, method: (opts && opts.method) || 'GET'});
    let body = {};
    const p = new URL(u).pathname;
    if (p === '/health') body = health;
    else if (p === '/api/mp/state') body = {mock: false};
    return Promise.resolve(new Response(JSON.stringify(body), {status: 200}));
  };
  w.eval(APP);
  await settle();
  return { dom, w, $, calls };
}

async function settle() { for (let i = 0; i < 6; i++) { await sleep(25); } }

async function click($, id, w) { assert.ok($(id), id); $(id).click(); await settle(); }

function fenceWrites(calls) {
  return calls.filter((c) => c.method !== 'GET' && /\/api\/(mp\/)?fence$/.test(c.url));
}

// 1. Served by the laptop bridge: the bridge is NOT a fence writer.
async function servedByBridge() {
  const { dom, w, $, calls } = await boot({
    origin: 'http://laptop.test:8080',
    health: {status:'ok', mock:true},     // the bridge's /health
  });
  assert.match($('fWriter').textContent, /BLOCKED/,
    'a bridge-served page must not claim a fence writer it has not verified');
  // the Pi URL box must NOT be pre-filled with the bridge's own address
  assert.notEqual($('piUrl').value, 'http://laptop.test:8080',
    'the bridge URL must never be pasted in as if it were the Pi');
  calls.length = 0;
  await click($, 'btnFenceApply', w);
  assert.equal(fenceWrites(calls).length, 0,
    'clicking UPLOAD FENCE on a bridge-served page must not fire a request');
  assert.match($('pConverted').textContent, /REFUSED/);
  // and it must tell the operator what to do about it
  assert.match($('fenceWriterHint').textContent, /Pi URL|Mission Planner/);
  // the deliberate opt-in still works, and is clearly labelled
  $('chkMpFenceFallback').checked = true;
  $('chkMpFenceFallback').dispatchEvent(new w.Event('change', {bubbles:true}));
  await settle();
  calls.length = 0;
  await click($, 'btnFenceApply', w);
  const mp = fenceWrites(calls);
  assert.equal(mp.length, 1, 'explicit fallback writes once, through MP');
  assert.equal(mp[0].url, 'http://laptop.test:8080/api/mp/fence');
  assert.match($('fWriter').textContent, /not direct/i);
  dom.window.close();
  console.log('PASS bridge-served UI: no fence writer assumed, refuse by default, MP only on opt-in');
}

// 2. Served BY the Pi: direct Pi -> Pixhawk is the writer, no pasting needed.
async function servedByPi() {
  const origin = 'http://192.168.29.21:8000';
  const { dom, w, $, calls } = await boot({
    origin,
    health: {service:'mission_pi', ok:true, link:true, cams:{}, ws:{}, fc:{device:'/dev/ttyACM0', armed:false}},
  });
  assert.match($('fWriter').textContent, /DIRECT Pi/,
    'the Pi serving the page is proof enough to own the fence');
  assert.equal($('piUrl').value, origin);
  calls.length = 0;
  await click($, 'btnFenceApply', w);
  const writes = fenceWrites(calls);
  assert.equal(writes.length, 1, 'exactly one writer');
  assert.equal(writes[0].url, origin + '/api/fence');
  assert.match($('fWriter').textContent, /DIRECT Pi/);
  dom.window.close();
  console.log('PASS Pi-served UI: direct Pi -> Pixhawk by default, no MP in the path');
}

// 3. Bridge page + ?pi=... : the direct path is opt-in per link, not per box.
async function bridgePageWithPiParam() {
  const pi = 'http://192.168.29.21:8000';
  const { dom, w, $, calls } = await boot({
    origin: 'http://laptop.test:8080',
    health: {status:'ok', mock:true},
    piUrlIsPi: (u) => u === pi,
    query: '?pi=' + encodeURIComponent(pi),
  });
  assert.equal($('piUrl').value, pi, '?pi= must fill the Pi box');
  assert.match($('fWriter').textContent, /DIRECT Pi/,
    'a verified Pi URL, even from a bridge-served page, owns the fence');
  calls.length = 0;
  await click($, 'btnFenceClear', w);
  assert.equal(fenceWrites(calls)[0].url, pi + '/api/fence');
  dom.window.close();
  console.log('PASS bridge page + ?pi= : direct Pi path with one verified writer');
}

async function main() {
  await servedByBridge();
  await servedByPi();
  await bridgePageWithPiParam();
}
main().then(() => process.exit(0)).catch((e) => { console.error(e); process.exit(1); });
