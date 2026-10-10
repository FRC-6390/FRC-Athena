/** Maps mechanism state / reason text to a semantic tone (design system: colour carries meaning only). */
export type Tone = 'ok' | 'warn' | 'fault' | 'info' | 'stale';

export function stateTone(state: string): Tone {
  const s = state.toLowerCase();
  if (s.includes('fault')) return 'fault';
  if (s.includes('blocked') || s.includes('starved') || s.includes('guard')) return 'warn';
  if (s.includes('running') || s.includes('setpoint') || s.includes('active')) return 'ok';
  if (s.includes('disabled') || s === '' || s === '--') return 'stale';
  return 'info';
}

/** The A5 State/Reason strings: Fault > Disabled > Blocked > Starved > Guard > Stage > Thermal > Power > Running > Idle. */
export function reasonTone(reason: string): Tone {
  const r = reason.toLowerCase();
  if (r.startsWith('fault')) return 'fault';
  if (r.startsWith('blocked') || r.startsWith('starved') || r.startsWith('guard') || r.startsWith('stage')
      || r.startsWith('thermal') || r.startsWith('power')) return 'warn';
  if (r.startsWith('disabled')) return 'stale';
  return 'ok';
}
