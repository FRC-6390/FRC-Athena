import { describe, expect, test } from 'bun:test';
import type { Sample, SignalValue } from '../../data/types';
import {
  groupCameras,
  notAcceptedWarning,
  visionAlertText,
  visionPath,
  visionRow,
  visionTone
} from './vision-status';

const sig = (path: string) => ({ path });

function values(map: Record<string, SignalValue>) {
  return (field: string) => map[field];
}

const seconds = (s: number) => s * 1_000_000;

describe('vision status grouping', () => {
  test('groups athena/vision paths into cameras by name, ignoring other paths', () => {
    const cameras = groupCameras([
      sig('athena/vision/front/State'),
      sig('athena/vision/front/Reason'),
      sig('athena/vision/back/State'),
      sig('athena/vision/State'),
      sig('athena/power/battery/voltage')
    ]);
    expect(cameras.map((c) => c.name)).toEqual(['back', 'front']);
    expect(Object.keys(cameras[1].signals).sort()).toEqual(['Reason', 'State']);
  });

  test('parses camera and field from a vision path', () => {
    expect(visionPath('athena/vision/front/FrameAgeMs')).toEqual({ camera: 'front', field: 'FrameAgeMs' });
    expect(visionPath('athena/vision/State')).toBeNull();
    expect(visionPath('robot/vision/front/State')).toBeNull();
  });
});

describe('vision state tone', () => {
  test('maps each published state to its tone', () => {
    expect(visionTone('OK')).toBe('ok');
    expect(visionTone('RECOVERING')).toBe('info');
    expect(visionTone('REJECTED')).toBe('warn');
    expect(visionTone('UNSYNCED')).toBe('warn');
    expect(visionTone('STALE')).toBe('stale');
    expect(visionTone('NO_TARGETS')).toBe('stale');
  });

  test('missing or empty state is stale; unknown text is info', () => {
    expect(visionTone(undefined)).toBe('stale');
    expect(visionTone('')).toBe('stale');
    expect(visionTone('SOMETHING_NEW')).toBe('info');
  });
});

describe('not accepted warning', () => {
  test('warns only when not accepted for more than 2 s while frames arrive', () => {
    expect(notAcceptedWarning(2.5, 40)).toBe(true);
    expect(notAcceptedWarning(2, 40)).toBe(false);
    expect(notAcceptedWarning(1.9, 40)).toBe(false);
    // Frames not arriving: STALE says it already, so no accepted-age warning.
    expect(notAcceptedWarning(5, 4000)).toBe(false);
    expect(notAcceptedWarning(null, 40)).toBe(false);
  });

  test('a stalled OK camera is shown as warn; a stalled NO_TARGETS camera too', () => {
    const stalled = visionRow('front', values({ State: 'OK', SecondsSinceAccepted: 3, FrameAgeMs: 30 }));
    expect(stalled.tone).toBe('warn');
    expect(stalled.stalled).toBe(true);
    expect(visionRow('front', values({ State: 'NO_TARGETS', SecondsSinceAccepted: 3, FrameAgeMs: 30 })).tone).toBe('warn');
    const fine = visionRow('front', values({ State: 'OK', SecondsSinceAccepted: 0.2, FrameAgeMs: 30 }));
    expect(fine.tone).toBe('ok');
    expect(fine.stalled).toBe(false);
  });
});

describe('vision row', () => {
  test('missing signals render as --', () => {
    const row = visionRow('front', values({}));
    expect(row.state).toBe('--');
    expect(row.reason).toBe('--');
    expect(row.tags).toBe('--');
    expect(row.acceptedPerSecond).toBe('--');
    expect(row.rejectedPerSecond).toBe('--');
    expect(row.frameAge).toBe('--');
    expect(row.sinceAccepted).toBe('--');
    expect(row.tone).toBe('stale');
    expect(row.clockNotSynced).toBe(false);
  });

  test('formats rates, frame age, tags and an empty OK reason', () => {
    const row = visionRow(
      'front',
      values({
        State: 'OK',
        Reason: '',
        AcceptedPerSecond: 12.34,
        RejectedPerSecond: 0,
        FrameAgeMs: 23.6,
        TagCount: 3n,
        SecondsSinceAccepted: 0.1
      })
    );
    expect(row.reason).toBe('');
    expect(row.acceptedPerSecond).toBe('12.3');
    expect(row.rejectedPerSecond).toBe('0.0');
    expect(row.frameAge).toBe('24 ms');
    expect(row.tags).toBe('3');
    expect(row.sinceAccepted).toBe('0.1 s');
  });

  test('TimeSync false shows the clock warning; true or absent does not', () => {
    expect(visionRow('front', values({ State: 'UNSYNCED', TimeSync: false })).clockNotSynced).toBe(true);
    expect(visionRow('front', values({ State: 'OK', TimeSync: true })).clockNotSynced).toBe(false);
    expect(visionRow('front', values({ State: 'OK' })).clockNotSynced).toBe(false);
  });
});

describe('vision driver alert', () => {
  const history = (state: string, times: number[]): Sample[] => times.map((t) => ({ timestampMicros: t, value: state }));

  test('UNSYNCED or STALE for more than 2 s gives a warn text', () => {
    const now = seconds(10);
    expect(visionAlertText('front', history('UNSYNCED', [seconds(6), seconds(9), now]), now)).toBe('front camera: UNSYNCED');
    expect(visionAlertText('back', history('STALE', [seconds(7), now]), now)).toBe('back camera: STALE');
  });

  test('a run of 2 s or less, a healthy state, or a run that broke recently gives no alert', () => {
    const now = seconds(10);
    expect(visionAlertText('front', history('UNSYNCED', [seconds(8), now]), now)).toBeNull();
    expect(visionAlertText('front', history('OK', [seconds(0), now]), now)).toBeNull();
    const broke: Sample[] = [
      { timestampMicros: seconds(1), value: 'UNSYNCED' },
      { timestampMicros: seconds(9), value: 'OK' },
      { timestampMicros: seconds(9.5), value: 'UNSYNCED' },
      { timestampMicros: now, value: 'UNSYNCED' }
    ];
    expect(visionAlertText('front', broke, now)).toBeNull();
    expect(visionAlertText('front', [], now)).toBeNull();
    expect(visionAlertText('front', history('UNSYNCED', [0]), null)).toBeNull();
  });
});
