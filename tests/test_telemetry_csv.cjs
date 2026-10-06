const assert = require('node:assert/strict');
const { test } = require('node:test');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const vm = require('node:vm');

// Exercise the production serializer without bootstrapping the dashboard DOM.
const source = readFileSync(join(__dirname, '../static/js/app.js'), 'utf8');
const serializer = source.slice(source.indexOf('  function telemetryToCsv()'),
  source.indexOf('  function buildReport('));
function csv(telemetryLog) {
  return vm.runInNewContext(`${serializer}\ntelemetryToCsv()`, { telemetryLog });
}

test('CSV includes fields introduced after stage separation', () => {
  assert.equal(csv([
    { time: 0, altitude: 0 },
    { time: 5, altitude: 100, booster_altitude: 90, booster_mass: 15000 },
  ]), 'time,altitude,booster_altitude,booster_mass\n0,0,,\n5,100,90,15000');
});

test('CSV escapes strings and handles missing and null values', () => {
  assert.equal(csv([
    { time: 0, phase: 'a,"b"\nc', optional: null, nested: {} },
    { time: 1, phase: 'done', optional: 5 },
  ]), 'time,phase,optional\n0,"a,""b""\nc",\n1,done,5');
  assert.equal(csv([]), '');
});
