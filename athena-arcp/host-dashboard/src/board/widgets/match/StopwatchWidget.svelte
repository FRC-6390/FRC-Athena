<script lang="ts">
  import { onDestroy } from 'svelte';
  import type { WidgetContext } from '../../../lib/board/types';
  import { STOPPED, elapsedMs, formatTenths, remainingMs, reset, start, stop, type Stopwatch } from '../../../lib/board/widgets/match-stopwatch';

  let { ctx }: { ctx: WidgetContext } = $props();

  const countdown = $derived(ctx.setting<string>('mode') === 'countdown');
  const target = $derived(ctx.setting<number>('target'));

  // Local runtime state: a stopwatch is not user configuration, so it is not saved.
  let sw = $state<Stopwatch>(STOPPED);
  let now = $state(performance.now());
  let ticker: ReturnType<typeof setInterval> | null = null;

  const running = $derived(sw.startedAt !== null);
  const value = $derived(countdown ? remainingMs(target, sw, now) : elapsedMs(sw, now));
  const finished = $derived(countdown && value === 0 && (sw.bankedMs > 0 || running));

  function syncTicker() {
    if (running && ticker === null) {
      ticker = setInterval(() => (now = performance.now()), 100);
    } else if (!running && ticker !== null) {
      clearInterval(ticker);
      ticker = null;
    }
  }

  $effect(() => {
    void running;
    syncTicker();
  });

  onDestroy(() => {
    if (ticker !== null) clearInterval(ticker);
  });

  function toggle() {
    now = performance.now();
    if (countdown && value === 0) {
      sw = reset();
      return;
    }
    sw = running ? stop(sw, now) : start(sw, now);
  }

  function clear() {
    sw = reset();
    now = performance.now();
  }

  const text = $derived(formatTenths(value));
</script>

<div class="watch" data-state={finished ? 'finished' : running ? 'running' : 'idle'}>
  <span class="mode">{countdown ? 'Countdown' : 'Stopwatch'}</span>
  <span class="time ui-mono">{text}</span>
  <div class="controls">
    <button type="button" class="go" onclick={toggle} disabled={ctx.editing}>
      {running ? 'Stop' : countdown && value === 0 ? 'Reset' : 'Start'}
    </button>
    <button type="button" class="clear" onclick={clear} disabled={ctx.editing || (!running && sw.bankedMs === 0)}>Reset</button>
  </div>
</div>

<style>
  .watch {
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-rows: auto 1fr auto;
    align-items: center;
    justify-items: center;
    padding: 6px 10px;
    box-sizing: border-box;
    container-type: size;
    color: var(--text-strong);
  }
  .mode {
    font-size: min(12cqh, 9cqw);
    font-weight: 700;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--text-soft);
  }
  .time {
    font-size: min(60cqh, 22cqw);
    font-weight: 700;
    line-height: 1;
    font-variant-numeric: tabular-nums;
  }
  .watch[data-state='running'] .time {
    color: var(--info);
  }
  .watch[data-state='finished'] .time {
    color: var(--fault);
  }
  .controls {
    display: flex;
    gap: 8px;
    width: 100%;
    justify-content: center;
  }
  button {
    min-height: 36px;
    min-width: 0;
    padding: 0 14px;
    border-radius: var(--radius);
    border: 1px solid var(--border-emphasis);
    background: var(--surface-3);
    color: var(--text-strong);
    font: inherit;
    font-size: min(max(14px, 5cqh), 20px);
    font-weight: 700;
    cursor: pointer;
  }
  button.go {
    flex: 1;
    max-width: 180px;
    background: var(--brand);
    border-color: var(--brand);
    color: var(--text-strong);
  }
  button.go:hover:not(:disabled) {
    background: var(--brand-strong);
  }
  button:disabled {
    opacity: 0.45;
    cursor: default;
  }
</style>
