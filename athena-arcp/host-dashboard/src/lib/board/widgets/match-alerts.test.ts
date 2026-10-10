import { describe, expect, test } from 'bun:test';
import { MARKER_KIND } from '../../data/types';
import { alertAge, buildAlerts, markerKey, signalKey, type MarkerInput } from './match-alerts';

const marker = (timestampMicros: number, kind: number, message: string): MarkerInput => ({
  timestampMicros,
  kind,
  message,
  label: 'label'
});

describe('alerts feed', () => {
  test('keeps fault, stage and check markers, newest first', () => {
    const { alerts } = buildAlerts({
      markers: [
        marker(1_000_000, MARKER_KIND.FAULT, 'Brownout'),
        marker(2_000_000, MARKER_KIND.AUTO_START, 'auto'),
        marker(3_000_000, MARKER_KIND.STAGE_CHANGE, 'turret: HOMING'),
        marker(4_000_000, MARKER_KIND.CHECK, 'pre-match check')
      ],
      faultSignals: [],
      dismissed: new Set(),
      limit: 10,
      showWarnings: true
    });
    expect(alerts.map((a) => a.text)).toEqual(['pre-match check', 'turret: HOMING', 'Brownout']);
    expect(alerts.map((a) => a.level)).toEqual(['info', 'warn', 'fault']);
  });

  test('hides warnings when disabled and respects the limit', () => {
    const { alerts } = buildAlerts({
      markers: [marker(1, MARKER_KIND.STAGE_CHANGE, 'a'), marker(2, MARKER_KIND.FAULT, 'b'), marker(3, MARKER_KIND.FAULT, 'c')],
      faultSignals: [],
      dismissed: new Set(),
      limit: 1,
      showWarnings: false
    });
    expect(alerts.map((a) => a.text)).toEqual(['c']);
  });

  test('active fault signals appear and inactive ones do not', () => {
    const { alerts } = buildAlerts({
      markers: [],
      faultSignals: [
        { path: 'robot/a/fault', active: true, timestampMicros: 5, name: 'Motor overheat' },
        { path: 'robot/b/fault', active: false, timestampMicros: 9, name: 'Encoder' }
      ],
      dismissed: new Set(),
      limit: 10,
      showWarnings: true
    });
    expect(alerts).toEqual([{ key: signalKey('robot/a/fault'), level: 'fault', text: 'Motor overheat', timestampMicros: 5 }]);
  });

  test('dismissed alerts stay hidden while live, and a cleared fault re-arms', () => {
    const fault = marker(7, MARKER_KIND.FAULT, 'Brownout');
    const dismissed = new Set([markerKey(fault), signalKey('robot/a/fault')]);
    const first = buildAlerts({
      markers: [fault],
      faultSignals: [{ path: 'robot/a/fault', active: false, timestampMicros: null, name: 'x' }],
      dismissed,
      limit: 10,
      showWarnings: true
    });
    expect(first.alerts).toHaveLength(0);
    // The signal cleared, so its dismissal is dropped; the marker is still live so it stays dismissed.
    expect([...first.dismissedNext]).toEqual([markerKey(fault)]);

    const again = buildAlerts({
      markers: [],
      faultSignals: [{ path: 'robot/a/fault', active: true, timestampMicros: 9, name: 'x' }],
      dismissed: first.dismissedNext,
      limit: 10,
      showWarnings: true
    });
    expect(again.alerts).toHaveLength(1);
  });

  test('age label is relative to the newest robot time', () => {
    expect(alertAge(10_000_000, 12_400_000)).toBe('2.4 s ago');
    expect(alertAge(0, 180_000_000)).toBe('3 m ago');
    expect(alertAge(null, 5)).toBe('');
  });
});
