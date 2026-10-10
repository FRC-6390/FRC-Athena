/**
 * Match clock rules for the match timer widget. Pure: the widget feeds in either a bound match-time signal
 * or the match markers, and gets back what to show.
 */
import { AUTO_SECONDS, TELEOP_SECONDS, type MatchPhase } from '../../driver/driver-state';

/** Full FRC match length: autonomous followed by teleop. */
export const MATCH_SECONDS = AUTO_SECONDS + TELEOP_SECONDS;

export type ClockPhase = 'idle' | 'auto' | 'teleop' | 'endgame' | 'final' | 'ended';
export type ClockTone = 'info' | 'warn' | 'fault' | null;

export type MatchClock = {
  phase: ClockPhase;
  /** Seconds left in the match, or null when idle. */
  remaining: number | null;
  tone: ClockTone;
  label: string;
};

export type ClockThresholds = {
  /** Endgame colour starts at or below this many seconds left. */
  endgameSeconds: number;
  /** Final colour starts at or below this many seconds left. */
  finalSeconds: number;
};

const PHASE_LABEL: Record<ClockPhase, string> = {
  idle: 'Idle',
  auto: 'Auto',
  teleop: 'Teleop',
  endgame: 'Endgame',
  final: 'Final',
  ended: 'Ended'
};

const IDLE: MatchClock = { phase: 'idle', remaining: null, tone: null, label: PHASE_LABEL.idle };

/** Clock from seconds elapsed since the match started (autonomous first). */
export function clockFromElapsed(elapsed: number, thresholds: ClockThresholds): MatchClock {
  if (!Number.isFinite(elapsed)) return IDLE;
  const e = Math.max(0, elapsed);
  if (e >= MATCH_SECONDS) return { phase: 'ended', remaining: 0, tone: null, label: PHASE_LABEL.ended };
  const remaining = MATCH_SECONDS - e;
  let phase: ClockPhase;
  let tone: ClockTone = null;
  if (e < AUTO_SECONDS) {
    phase = 'auto';
    tone = 'info';
  } else if (remaining <= thresholds.finalSeconds) {
    phase = 'final';
    tone = 'fault';
  } else if (remaining <= thresholds.endgameSeconds) {
    phase = 'endgame';
    tone = 'warn';
  } else {
    phase = 'teleop';
  }
  return { phase, remaining, tone, label: PHASE_LABEL[phase] };
}

/** Clock from a signal that counts down (`remaining` seconds left in the match). */
export function clockFromRemaining(remaining: number, thresholds: ClockThresholds): MatchClock {
  if (!Number.isFinite(remaining)) return IDLE;
  return clockFromElapsed(MATCH_SECONDS - remaining, thresholds);
}

/**
 * Clock from the marker-derived phase. `sinceSeconds` is seconds since that phase started (null when idle).
 * Teleop starts after the autonomous period, so its elapsed time is offset by AUTO_SECONDS.
 */
export function clockFromPhase(phase: MatchPhase, sinceSeconds: number | null, thresholds: ClockThresholds): MatchClock {
  if (phase === 'ended') return clockFromElapsed(MATCH_SECONDS, thresholds);
  if (phase === 'idle' || sinceSeconds === null) return IDLE;
  if (phase === 'auto') return clockFromElapsed(sinceSeconds, thresholds);
  return clockFromElapsed(AUTO_SECONDS + sinceSeconds, thresholds);
}

/** m:ss, rounding up so the display never reads 0:00 while time is left. */
export function formatMatchClock(seconds: number | null): string {
  if (seconds === null || !Number.isFinite(seconds)) return '--:--';
  const total = Math.ceil(Math.max(0, seconds));
  return `${Math.floor(total / 60)}:${String(total % 60).padStart(2, '0')}`;
}
