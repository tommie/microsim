export default function(opts?: {
  print: typeof console.log,
  printErr: typeof console.error,
}): Promise<typeof Microsim>;

declare namespace Microsim {

  // --- core ---

  // A point in time, in simulator milliseconds.
  type SimTimePoint = number;

  // A point in time of some Clock, in its clock ticks.
  type ClockTimePoint = number;

  // A duration, in simulator milliseconds.
  type SimDuration = number;

  // A duration of some Clock, in its clock ticks.
  type ClockDuration = number;

  interface Advancement {
    nextTime: SimTimePoint;
  }

  class AdvancementLimit {
    constructor(proto: Partial<AdvancementLimit>);

    canAdvanceTo(at: SimTimePoint): boolean;
    advanced(at: SimTimePoint): void;
  }

  class Clock {
    constructor(interval: SimDuration);

    interval: SimDuration;
    now: SimTimePoint;
    at(at: ClockTimePoint): SimTimePoint;
    advanceTo(at: SimTimePoint): void;
  }

  class DeviceListener {
    static implement<T extends Partial<DeviceListener>>(obj: T): T & DeviceListener;

    invalidInternalState(status: Status): void;
    pinChanged(pin: Pin, kind: PinChange): void;
  }

  class Device extends SimulationObject {
    pins: PinDescriptor[];
  }

  class Pin {
    value: number;
    resistance: number;
    setExternal(v: number): void;
  }

  enum PinChange {
    VALUE,
    RESISTANCE,
  }

  class PinDescriptor {
    name: string;
    path: string[];
    getPin(): Pin;
  }

  class Schedulable {
    advanceTo(limit: AdvancementLimit): Advancement;
  }

  class SimulationContext {
    constructor();

    addClock(clock: Clock): void;
    addObject(obj: SimulationObject): void;
    makeSimulator(): Simulator;
  }

  class SimulationObject extends Schedulable {
    static implement<T extends Partial<SimulationObject>>(obj: T): T & SimulationObject;

    getClockSources(): Clock[];
  }

  class Simulator {
    advanceTo(limit: AdvancementLimit): Advancement;
  }


  // --- pic14 ---

  enum EUSARTDataTraceEntryMode {
    ASYNC_TRANSMIT,
    ASYNC_RECEIVED,
    SYNC_MASTER_TRANSMIT,
    SYNC_MASTER_RECEIVED,
    SYNC_SLAVE_TRANSMIT,
    SYNC_SLAVE_RECEIVED,
  }

  class Pic14ICSP {
    release(): void;
    loadProgram(addr: number, data: Uint8Array): void;
    loadData(addr: number, data: Uint8Array): void;
  }

  class P16F887 extends Device {
    constructor(listener: DeviceListener, extosc: Clock);

    enterICSP(): Pic14ICSP;

    sleeping: boolean;
  }


  // --- util ---

  class Status {
    error: string;
    context?: string;
    ok: boolean;
  }

}
