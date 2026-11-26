"use strict";

const assert = require("assert");
require("../static/app.js");

const geometry = globalThis.LidarGeometry;
const centerX = 100;
const centerY = 100;
const forward = geometry.polarToCanvas(2, 0, 10, centerX, centerY);
assert.strictEqual(forward.x, 100);
assert.strictEqual(forward.y, 80);

const left = geometry.polarToCanvas(2, Math.PI / 2, 10, centerX, centerY);
assert.ok(Math.abs(left.x - 80) < 1e-9);
assert.ok(Math.abs(left.y - 100) < 1e-9);
assert.strictEqual(geometry.effectiveSourceAge(0.1, 0.2, 0.19) < 0.5, true);
assert.strictEqual(geometry.effectiveSourceAge(0.1, 0.2, 0.21) > 0.5, true);
assert.strictEqual(geometry.effectiveSourceAge(null, 0, 0), Infinity);

const angleMin = -1.647495366;
const angleIncrement = 0.00306796166;
assert.strictEqual(geometry.geometryMatches({
  ranges: new Array(1075).fill(null),
  geometry: {
    angle_min: angleMin,
    angle_max: angleMin + 1074 * angleIncrement,
    angle_increment: angleIncrement
  }
}), true);

console.log("UI geometry tests passed");
