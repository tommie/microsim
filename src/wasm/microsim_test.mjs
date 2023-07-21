"use strict";

import * as assert from "node:assert";
import { afterEach, test } from "node:test";

import loadMicrosim_ from "./microsim.mjs";

async function loadMicrosim(timeoutMS) {
  const microsim = await loadMicrosim_({
    print: console.log,
    printErr: console.error,
  });

  // This must be set before the first time the trace buffer is
  // requested.
  microsim.TraceBuffer.setGlobalCapacity(1 << 10);

  return microsim;
}

function vectorToArray(v) {
  const out = [];
  for (let i = 0; i < v.size(); ++i) {
    out.push(v.get(i));
  }
  return out;
}

function logTraceBuffer(buffer) {
  while (buffer.length > 0) {
    console.log("Trace: ", buffer.top.getDecoded());
    buffer.pop();
  }
}

test("microsim", async () => {
  await test("loads", async () => {
    await loadMicrosim();
  });
});

test("core", async () => {
  const microsim = await loadMicrosim();

  await test("Clock can be used", () => {
    const osc = new microsim.Clock(1000);
    assert.equal(osc.interval, 1000);
    assert.equal(osc.now, 0);
  });

  await test("AdvancementLimit can be used", () => {
    let now;
    const limit = new microsim.AdvancementLimit({
      canAdvanceTo(tp) { return tp > 0; },
      advanced(tp) { now = tp; },
    });

    assert.equal(limit.canAdvanceTo(0), false);
    assert.equal(limit.canAdvanceTo(1), true);

    limit.advanced(2);
    assert.equal(now, 2);
  });

  await test("AdvancementLimit can be empty", () => {
    const limit = new microsim.AdvancementLimit({});
    assert.equal(limit.canAdvanceTo(0), true);
    limit.advanced(0);
  });

  await test("SimulationContext can be used", () => {
    const simctx = new microsim.SimulationContext();
    const sim = simctx.makeSimulator();
    const adv = sim.advanceTo(new microsim.AdvancementLimit({}));

    assert.equal(adv.nextTime, Infinity);
  });

  await test("TraceBuffer can be used", () => {
    const buffer = microsim.TraceBuffer.getGlobal();
    while (buffer.length) {
      buffer.pop();
    }

    const clock = new microsim.Clock(1);
    clock.advanceTo(42);

    assert.equal(buffer.discarded, 0, "bad TraceBuffer.discarded");
    assert.equal(buffer.length, 1, "bad TraceBuffer.length");

    const entry = buffer.top.getDecoded();
    assert.ok(entry instanceof microsim.ClockAdvancedTraceEntry);
    assert.equal(entry.now, 42);
    buffer.pop();
  });
});

test("pic14", async () => {
  const microsim = await loadMicrosim();

  function makeProcessorSimulator(proc, osc) {
    const simctx = new microsim.SimulationContext();
    simctx.addClock(osc);
    simctx.addObject(proc);

    const sim = simctx.makeSimulator();
    sim.$ctx = simctx;  // Save a reference, since simctx may own objects.

    sim.advanceWhile = function advanceWhile(pred) {
      while (pred()) {
        const adv = this.advanceTo(new microsim.AdvancementLimit({
          canAdvanceTo() { return pred(); },
        }));
        if (!pred()) return adv;

        if (adv.nextTime === Infinity) {
          throw new Error("Simulation has stalled");
        }
      }
    };

    return sim;
  }

  //afterEach(() => logTraceBuffer(microsim.TraceBuffer.getGlobal()));

  await test("P16F887 basics work", async () => {
    const invalidInternalStates = [];
    const pinChanges = [];
    const deviceListener = microsim.DeviceListener.implement({
      invalidInternalState(status) {
        invalidInternalStates.push(status);
      },

      pinChanged(pin, kind) {
        pinChanges.push([pin, kind.constructor.name]);
      },
    });

    const osc = new microsim.Clock(1);
    const proc = new microsim.P16F887(deviceListener, osc);

    const pinDescrs = new Map(vectorToArray(proc.pins).map((descr) => [descr.name, descr]));
    const pinDescrsByPtr = new Map(vectorToArray(proc.pins).map((descr) => [descr.getPin().$ptr, descr]));
    assert.ok(pinDescrs.size >= 61);

    const ra0Descr = pinDescrs.get("RA0");
    assert.equal(ra0Descr.name, "RA0");
    assert.deepEqual(vectorToArray(ra0Descr.path), []);
    const ra0 = ra0Descr.getPin();
    assert.ok(isNaN(ra0.value), "bad ra0.value");
    assert.equal(ra0.resistance, Infinity, "bad ra0.resistance");
    ra0.setExternal(1);

    const icsp = proc.enterICSP();
    try {
      // Assign RA0 = RB0
      // Sleep
      icsp.loadProgram(0, new Uint16Array([0xFE, 0x30, 0x83, 0x16, 0x03, 0x13, 0x85, 0x00, 0x83, 0x12, 0x03, 0x13, 0x06, 0x08, 0x83, 0x12, 0x03, 0x13, 0x85, 0x00, 0x63, 0x00]));
    } finally {
      icsp.release();
    }

    assert.ok(!proc.sleeping);

    pinDescrs.get("RB0").getPin().setExternal(1);

    const sim = makeProcessorSimulator(proc, osc);
    sim.advanceWhile(() => !proc.sleeping);

    assert.deepEqual(invalidInternalStates.map((status) => status.constructor.name), []);
    assert.deepEqual(
      pinChanges.map((change) => [pinDescrsByPtr.get(change[0].$ptr).name, change[1]]),
      [
        ["RA0", microsim.PinChange.RESISTANCE.constructor.name],
        ["RA0", microsim.PinChange.VALUE.constructor.name],
      ]);

    proc.delete();
  });
});
