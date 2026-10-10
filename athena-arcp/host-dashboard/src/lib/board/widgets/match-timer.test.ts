import { describe, expect, test } from 'bun:test';
import { clockFromElapsed, clockFromPeriod, clockFromPhase, clockFromRemaining, formatMatchClock, MATCH_SECONDS } from './match-timer';
import { parseAlliance, parseStation } from './match-alliance';
import { batteryReading, batteryTone, sparklinePoints } from './match-battery';

const T = { endgameSeconds: 30, finalSeconds: 15 };

describe('match clock', () => {
  test('phases follow the auto and endgame thresholds', () => {
    expect(clockFromElapsed(0, T).phase).toBe('auto');
    expect(clockFromElapsed(14.9, T).phase).toBe('auto');
    expect(clockFromElapsed(20, T).phase).toBe('teleop');
    expect(clockFromElapsed(MATCH_SECONDS - 31, T).phase).toBe('teleop');
    expect(clockFromElapsed(MATCH_SECONDS - 30, T)).toMatchObject({ phase: 'endgame', tone: 'warn' });
    expect(clockFromElapsed(MATCH_SECONDS - 15, T)).toMatchObject({ phase: 'final', tone: 'fault' });
    expect(clockFromElapsed(MATCH_SECONDS, T)).toMatchObject({ phase: 'ended', remaining: 0 });
  });

  test('non-finite or missing time is idle', () => {
    expect(clockFromElapsed(Number.NaN, T)).toMatchObject({ phase: 'idle', remaining: null, tone: null });
    expect(clockFromPhase('idle', null, T).phase).toBe('idle');
  });

  test('a remaining-time signal is converted to the same clock', () => {
    expect(clockFromRemaining(10, T)).toMatchObject({ phase: 'final', remaining: 10 });
    expect(clockFromRemaining(140, T).phase).toBe('auto');
  });

  test('marker phases offset teleop by the autonomous period', () => {
    expect(clockFromPhase('auto', 3, T)).toMatchObject({ phase: 'auto', remaining: MATCH_SECONDS - 3 });
    expect(clockFromPhase('teleop', 0, T)).toMatchObject({ phase: 'teleop', remaining: MATCH_SECONDS - 15 });
    expect(clockFromPhase('ended', 4, T).phase).toBe('ended');
  });

  test('formats m:ss rounding up', () => {
    expect(formatMatchClock(134.2)).toBe('2:15');
    expect(formatMatchClock(0)).toBe('0:00');
    expect(formatMatchClock(null)).toBe('--:--');
  });
});

describe('alliance and station', () => {
  test('parses strings and booleans', () => {
    expect(parseAlliance('Red')).toBe('red');
    expect(parseAlliance('BLUE ALLIANCE')).toBe('blue');
    expect(parseAlliance('green')).toBeNull();
    expect(parseAlliance(true)).toBe('red');
    expect(parseAlliance(false)).toBe('blue');
    expect(parseAlliance(true, 'blue')).toBe('blue');
  });

  test('accepts stations 1 to 3 only', () => {
    expect(parseStation(2)).toBe(2);
    expect(parseStation(BigInt(3))).toBe(3);
    expect(parseStation(0)).toBeNull();
    expect(parseStation(4)).toBeNull();
    expect(parseStation(1.5)).toBeNull();
  });
});

describe('battery', () => {
  test('tone thresholds', () => {
    expect(batteryTone(12.6, 12, 11)).toBe('ok');
    expect(batteryTone(11.9, 12, 11)).toBe('warn');
    expect(batteryTone(10.8, 12, 11)).toBe('fault');
    expect(batteryTone(null, 12, 11)).toBeNull();
  });

  test('reads newest voltage, window minimum and relative sag history', () => {
    const samples = [
      { timestampMicros: 0, value: 12.8 },
      { timestampMicros: 10_000_000, value: 11.4 },
      { timestampMicros: 20_000_000, value: 11.9 }
    ];
    const reading = batteryReading(samples, { warnBelow: 12, faultBelow: 11, windowSeconds: 15 });
    expect(reading.volts).toBe(11.9);
    expect(reading.min).toBe(11.4);
    expect(reading.tone).toBe('warn');
    expect(reading.points.map((p) => p.seconds)).toEqual([-10, 0]);
  });

  test('ignores non-numeric samples and reports nothing without data', () => {
    expect(batteryReading([{ timestampMicros: 1, value: 'x' }], { warnBelow: 12, faultBelow: 11, windowSeconds: 60 }).volts).toBeNull();
  });

  test('sparkline is empty below two points and scales into the box', () => {
    expect(sparklinePoints([{ seconds: 0, volts: 12 }], 100, 40)).toBe('');
    const line = sparklinePoints(
      [
        { seconds: -10, volts: 12 },
        { seconds: 0, volts: 10 }
      ],
      100,
      40
    );
    expect(line.split(' ')).toHaveLength(2);
    expect(line.startsWith('2.0,')).toBe(true);
  });
});

describe('clockFromPeriod (WPILib match time)', () => {
  const t = { endgameSeconds: 30, finalSeconds: 10 };
  test('the DriverStation mode picks the period, match time is what is left in it', () => {
    expect(clockFromPeriod('Autonomous', 12, t)).toMatchObject({ phase: 'auto', remaining: 12 });
    expect(clockFromPeriod('Teleop', 100, t)).toMatchObject({ phase: 'teleop', remaining: 100 });
    expect(clockFromPeriod('Teleop', 25, t)).toMatchObject({ phase: 'endgame', tone: 'warn' });
    expect(clockFromPeriod('Teleop', 5, t)).toMatchObject({ phase: 'final', tone: 'fault' });
  });
  test('outside a timed match there is no time, only the mode', () => {
    expect(clockFromPeriod('Teleop', -1, t)).toMatchObject({ phase: 'teleop', remaining: null });
    expect(clockFromPeriod('Disabled', -1, t)).toMatchObject({ phase: 'idle', label: 'Disabled', remaining: null });
  });
});
