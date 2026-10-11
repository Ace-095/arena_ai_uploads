/* Browser-side geofence test: the REAL buttons, against a Pi whose "Pixhawk"
 * is tools/fake_sitl.py (started by mission_pi/tests/test_fence_e2e_fake_fc.py).
 *
 * What this proves: clicking UPLOAD FENCE / refresh / clear in the shipped UI
 * sends a request to <pi>/api/fence and the UI's verdict matches what the
 * flight controller really did. The Python test then asserts the fake FC's own
 * state, closing the loop browser -> Pi -> MAVLink -> FC.
 *
 *   node tests/test_fence_dom_to_fake_fc.cjs http://127.0.0.1:<port>
 */
const assert = require('node:assert/strict');
const { createHarness } = require('./harness.cjs');

const base = process.argv[2];
if (!base) throw new Error('Use the Python fixture, not a live flight controller');

async function main() {
  const h = createHarness(base);
  const { $, calls, vertices, setPolygon, boot, settle, click, writes, lastBody,
          fault, caps } = h;
  await boot();

  // connect the Pi link (the only writer this page may use)
  $('piUrl').value = base;
  await click('btnPiConnect');
  assert.match($('fWriter').textContent, /DIRECT Pi/, 'the Pi must own the fence writer');

  // 1. upload the drawn polygon
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(writes().length, 1, 'exactly one writer');
  assert.equal(writes()[0].url, base + '/api/fence');
  assert.deepEqual(lastBody().vertices, vertices);
  assert.match($('fReadback').textContent, /MATCH/, 'the FC readback must be shown as verified');
  assert.match($('fState').textContent, /LOADED on FC/);
  // the panel must state what the FC itself advertises BEFORE an upload
  assert.match($('fCapability').textContent, /fw 4\.5\.7/,
               'the FC firmware version must be visible');
  assert.match($('fCapability').textContent, /MAVLink 2/);
  assert.match($('fCapability').textContent, /fence mission type OK/);
  assert.equal($('fVerts').textContent, '4');
  assert.match($('pConverted').textContent, /FC verified: 4 vertices/);
  console.log('PASS DOM: upload -> Pi -> FC, readback MATCH shown');

  // 2. refresh reads the FC (source of truth), not the drawing
  calls.length = 0;
  await click('btnFenceRefresh');
  assert.equal(calls.length, 1);
  assert.equal(calls[0].url, base + '/api/fence');
  assert.match($('fReadback').textContent, /MATCH/);

  // 3. an FC that ACKs but does not store must read as FAILURE in the UI
  await fault('upload');           // the fake denies the transfer
  calls.length = 0;
  await click('btnFenceApply');
  assert.match($('pConverted').textContent, /NOT confirmed/);
  assert.notEqual($('fState').textContent, 'LOADED on FC',
                  'a failed upload must not leave the panel saying LOADED');
  assert.equal($('btnFenceApply').disabled, false, 'the button must come back');
  console.log('PASS DOM: FC rejection -> NOT confirmed, no false LOADED');

  // 4. a lying FC (ACK accepted, vertices dropped) must also read as FAILURE.
  //    The drawing must DIFFER from what the FC already holds, otherwise a
  //    discarded upload leaves the old (identical) fence and the readback
  //    legitimately matches — this is the case a naive ACK check cannot catch.
  const drawn2 = [{lat:15.3650,lon:75.1230},{lat:15.3650,lon:75.1234},
                  {lat:15.3654,lon:75.1234},{lat:15.3654,lon:75.1230}];
  setPolygon(drawn2);
  await fault('readback');
  calls.length = 0;
  await click('btnFenceApply');
  assert.deepEqual(lastBody().vertices, drawn2, 'the new drawing must be what was sent');
  assert.match($('pConverted').textContent, /NOT confirmed/);
  assert.notEqual($('fState').textContent, 'LOADED on FC');
  console.log('PASS DOM: acked-then-discarded -> NOT confirmed');

  // 5. clear goes through the Pi as well
  await fault(null);
  calls.length = 0;
  await click('btnFenceClear');
  assert.equal(calls.filter((c) => c.url.endsWith('/api/fence')).length, 1);
  assert.equal(calls[0].method, 'DELETE');
  assert.match($('pConverted').textContent, /cleared \(FC readback/);
  assert.equal($('fVerts').textContent, '0');
  console.log('PASS DOM: clear -> Pi -> FC, readback confirmed');

  // 6. A Pixhawk 2.4.8 on old firmware: MAVLink 2 link, but the firmware does
  //    not implement the fence mission type. The panel must SAY SO before the
  //    operator presses UPLOAD — and the UI must not silently over-block.
  await caps({caps: 8192});                 // MAVLINK2 only, no MISSION_FENCE
  calls.length = 0;
  await click('btnFenceRefresh');
  assert.match($('fCapability').textContent, /MAVLink 2/);
  assert.match($('fCapability').textContent, /NO fence mission type/,
               'an FC without fence support must be flagged');
  assert.match($('fCapability').style.color, /rgb\(255, 107, 107\)/,
               'the warning must be visually distinct');
  calls.length = 0;
  await click('btnFenceApply');
  assert.equal(calls.filter((c) => c.url.endsWith('/api/fence')).length, 1,
               'the operator may still attempt it — warn, never silently block');
  console.log('PASS DOM: old-firmware FC flagged before upload, not over-blocked');
  // put the fixture back where the earlier steps left it (FC holding nothing),
  // which is also the invariant the Python side asserts afterwards
  calls.length = 0;
  await click('btnFenceClear');
  assert.equal($('fVerts').textContent, '0');

  // 7. A silent FC (no AUTOPILOT_VERSION at all) reads as unknown, not as OK
  //    and not as a hard failure.
  await caps({caps: null, silent: true});
  await click('btnFenceRefresh');
  assert.match($('fCapability').textContent, /^unknown/);
  console.log('PASS DOM: silent FC -> "unknown", never a false green');
  await caps({caps: null, silent: false});

  h.dom.window.close();
  console.log('DONE fence-dom');
}

main().then(() => process.exit(0)).catch((e) => { console.error(e); process.exit(1); });
