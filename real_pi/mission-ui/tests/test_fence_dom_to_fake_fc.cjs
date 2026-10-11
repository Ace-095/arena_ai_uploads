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
  const { $, calls, vertices, setPolygon, boot, settle, click, writes, lastBody, fault } = h;
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

  h.dom.window.close();
  console.log('DONE fence-dom');
}

main().then(() => process.exit(0)).catch((e) => { console.error(e); process.exit(1); });
