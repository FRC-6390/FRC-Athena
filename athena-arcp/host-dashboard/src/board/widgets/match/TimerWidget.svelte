<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { matchPhase, robotNowMicros } from '../../../lib/driver/driver-state';
  import { ROBOT_PATHS } from '../../../lib/data/types';
  import {
    clockFromPeriod,
    clockFromPhase,
    clockFromRemaining,
    clockFromElapsed,
    formatMatchClock,
    type ClockThresholds
  } from '../../../lib/board/widgets/match-timer';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.time?.[0]);
  const source = $derived(ctx.setting<string>('source'));
  const thresholds = $derived<ClockThresholds>({
    endgameSeconds: ctx.setting<number>('endgame'),
    finalSeconds: ctx.setting<number>('final')
  });

  /** Bound signal wins; otherwise the match markers drive the clock. */
  const clock = $derived.by(() => {
    void ctx.live.current;
    const value = signal ? ctx.latest(signal) : undefined;
    const n = typeof value === 'number' ? value : typeof value === 'bigint' ? Number(value) : null;
    if (signal && n !== null) {
      if (source === 'period') {
        const modeSignal = ctx.session.store.descriptorByPath(ROBOT_PATHS.mode);
        const mode = modeSignal ? ctx.latest(modeSignal) : undefined;
        return clockFromPeriod(typeof mode === 'string' ? mode : null, n, thresholds);
      }
      return source === 'remaining' ? clockFromRemaining(n, thresholds) : clockFromElapsed(n, thresholds);
    }
    const store = ctx.session.store;
    const { phase, since } = matchPhase(store);
    const now = robotNowMicros(store);
    const sinceSeconds = since !== null && now !== null ? Math.max(0, (now - since) / 1_000_000) : null;
    return clockFromPhase(phase, sinceSeconds, thresholds);
  });

  const text = $derived(formatMatchClock(clock.remaining));
</script>

<div class="timer" data-tone={clock.tone}>
  <span class="phase">{clock.label}</span>
  <span class="clock ui-mono">{text}</span>
</div>

<style>
  .timer {
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-rows: auto 1fr;
    align-items: center;
    justify-items: center;
    padding: 4px 8px;
    container-type: size;
    color: var(--text-strong);
  }
  .phase {
    font-size: min(16cqh, 9cqw);
    font-weight: 700;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--text-soft);
    line-height: 1;
  }
  .clock {
    font-size: min(70cqh, 27cqw);
    font-weight: 700;
    line-height: 1;
    letter-spacing: -0.02em;
    font-variant-numeric: tabular-nums;
  }
  .timer[data-tone='info'] .phase,
  .timer[data-tone='info'] .clock {
    color: var(--info);
  }
  .timer[data-tone='warn'] .phase,
  .timer[data-tone='warn'] .clock {
    color: var(--warn);
  }
  .timer[data-tone='fault'] .phase,
  .timer[data-tone='fault'] .clock {
    color: var(--fault);
  }
</style>
