/* harness.cjs — the real index.html + app.js driven by real DOM events.
 *
 * Only the browser is simulated: the page, the scripts, the button wiring and
 * the fetch() calls are the shipped ones. Every request that targets the Pi
 * under test (`baseUrl`) is forwarded to it for real; everything else (map
 * tiles, the MP bridge) is stubbed, because those are different subsystems.
 *
 * Used by:
 *   test_commands_dom.cjs        — full UI surface against a Pi with cameras
 *   test_fence_dom_to_fake_fc.cjs — fence clicks against a Pi with a fake FC
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { JSDOM } = require('jsdom');

const root = path.resolve(__dirname, '..');

function createHarness(baseUrl) {
  if (!baseUrl) throw new Error('createHarness needs the Pi base URL (do not point it at a live aircraft)');
  const dom = new JSDOM(fs.readFileSync(path.join(root, 'index.html'), 'utf8'), {
    url: 'http://laptop.test', runScripts: 'outside-only', pretendToBeVisual: true,
  });
  const w = dom.window;
  const $ = (id) => w.document.getElementById(id);
  const calls = [];
  const pending = new Set();
  const noop = () => {};

  // polygon the "map" would hand to the fence conversion. Mutated IN PLACE by
  // setPolygon so every closure (map -> app) sees the new drawing.
  const vertices = [{lat:0,lon:0},{lat:0,lon:.001},{lat:.001,lon:.001},{lat:.001,lon:0}];
  function setPolygon(next) {
    vertices.splice(0, vertices.length, ...next.map((v) => ({...v})));
    return vertices;
  }
  const fakeMap = new Proxy({
    getPolygonVertices: () => vertices, getVertices: () => vertices,
    polygonToFence: () => vertices,
  }, {get: (obj, k) => obj[k] || noop});
  w.MissionMap = {initMap: () => fakeMap};

  // Pi link: the socket only opens for the Pi under test
  let connectedUrl = null;
  w.MissionLinks = {
    initPiLink: (opts) => ({
      connect: (url) => { connectedUrl = url; opts.onStatus({connected: url === baseUrl, url, retry: 0}); },
      close: noop,
      // Same contract as the real links.js: the probe result is delivered
      // through onProbe, which is what lets the UI verify the writer.
      probe: async () => {
        const res = connectedUrl === baseUrl
          ? {ok: true, isPi: true, ms: 2, role: 'mission_pi',
             health: {service: 'mission_pi', cams: {}, link: true, ws: {}}}
          : {ok: false, ms: 4000, error: 'unreachable'};
        opts.onProbe && opts.onProbe(res);
        return res;
      },
    }),
    initBridge: () => ({connect: noop}),
  };
  w.MissionCameras = {initCameras: () => new Proxy({}, {get: () => noop})};
  w.confirm = () => true;

  w.fetch = (url, opts = {}) => {
    const u = new URL(url, w.location.href).href;
    calls.push({url: u, ...opts});
    let p;
    if (u.startsWith(baseUrl)) p = fetch(u, opts);
    else {
      const d = u.endsWith('/api/mp/state') ? {mock: false} : {};
      p = Promise.resolve(new Response(JSON.stringify(d), {status: 200}));
    }
    pending.add(p);
    p.finally(() => pending.delete(p));
    return p;
  };

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  async function settle() {
    // let response.json() + handler promise chains + debounces finish
    for (let i = 0; i < 5; i++) { await Promise.allSettled([...pending]); await sleep(30); }
  }
  async function click(id) { assert.ok($(id), 'missing element #' + id); $(id).click(); await settle(); }
  function writes() {
    return calls.filter((c) => c.method && c.method !== 'GET' && !c.url.endsWith('/api/camera/status'));
  }
  function lastBody() { return JSON.parse(writes().at(-1).body); }
  async function fault(fail) {
    // Test-only endpoint present in the Python fixtures, never in production.
    // `fault` is the documented key; `fail` is accepted by the fixtures too so
    // an older script cannot silently fail to inject anything.
    const r = await fetch(baseUrl + '/__test/fault', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({fault: fail, fail}),
    });
    if (!r.ok) throw new Error('fault injection rejected: HTTP ' + r.status);
    return r.json();
  }
  async function caps(body) {
    // Test-only: change what the FIXTURE flight controller advertises
    // (firmware capability bits / silence), so the UI's "can this FC take a
    // fence at all?" line can be exercised. Never present in production.
    const r = await fetch(baseUrl + '/__test/caps', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(body || {}),
    });
    if (!r.ok) throw new Error('caps injection rejected: HTTP ' + r.status);
    return r.json();
  }
  function boot() {
    w.eval(fs.readFileSync(path.join(root, 'js/app.js'), 'utf8'));
    return settle();
  }

  return {dom, w, $, calls, vertices, setPolygon, boot, settle, click,
          writes, lastBody, fault, caps};
}

module.exports = {createHarness};
