<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { confirmReason, describeChange, finiteSetting, nudge, safeRange } from '../../../lib/board/widgets/controls-numeric';
  import Feedback from './parts/Feedback.svelte';
  import ConfirmBar from './parts/ConfirmBar.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const robot = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null;
  });
  const step = $derived(finiteSetting(ctx.setting<number>('step')) ?? 1);
  const userMin = $derived(ctx.setting<number>('min'));
  const userMax = $derived(ctx.setting<number>('max'));
  // Steps stay inside the safe range (user bounds, else 0 to max(1, 4 x |value|)).
  const range = $derived(safeRange(robot, userMin, userMax));
  const locked = $derived.by(() => {
    void ctx.live.current;
    return signal ? ctx.locked(signal) : false;
  });
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = signal ? ctx.reply(signal) : undefined;
    return r && r.op === 'write' ? r : undefined;
  });
  const now = $derived.by(() => {
    void ctx.live.current;
    return Date.now();
  });

  // While a write is in flight, the next press steps from the value being sent so quick taps accumulate.
  let pending = $state<number | null>(null);
  let confirm = $state<{ value: number; reason: string } | null>(null);
  const base = $derived(pending ?? robot ?? 0);

  function reasonText(value: number, reason: string): string {
    if (reason === 'range') return `Send ${value}? It is outside the limits ${userMin ?? ''} to ${userMax ?? ''}.`;
    if (reason === 'unknown') return `Send ${value}? The robot value is unknown, so the change cannot be checked.`;
    return `Send ${describeChange(robot, value)}?`;
  }

  async function send(value: number) {
    if (!signal) return;
    pending = value;
    try {
      await ctx.write(signal, value);
    } finally {
      pending = null;
      confirm = null;
    }
  }

  function press(direction: 1 | -1) {
    if (!signal || ctx.editing) return;
    const value = nudge(base, direction, step, range);
    const reason = confirmReason(robot, value, { userMin, userMax });
    if (reason) {
      confirm = { value, reason };
      return;
    }
    void send(value);
  }
</script>

<div class="nudge">
  <button type="button" class="ui-btn step" aria-label="Decrease" disabled={!signal?.writable || ctx.editing} onclick={() => press(-1)}>−</button>
  <div class="readout">
    <span class="value ui-mono">{formatNumber(base)}{#if signal?.unit}<small>{signal.unit}</small>{/if}</span>
    <span class="robot ui-mono">{robot === null ? 'robot --' : `robot ${formatNumber(robot)}`} · step {step}</span>
    <Feedback {reply} {locked} pending={pending !== null} {now} />
  </div>
  <button type="button" class="ui-btn step" aria-label="Increase" disabled={!signal?.writable || ctx.editing} onclick={() => press(1)}>+</button>
  {#if confirm}
    <div class="confirm">
      <ConfirmBar text={reasonText(confirm.value, confirm.reason)} onconfirm={() => confirm && send(confirm.value)} oncancel={() => (confirm = null)} />
    </div>
  {/if}
</div>

<style>
  .nudge {
    display: grid;
    grid-template-columns: minmax(40px, 1fr) minmax(0, 2fr) minmax(40px, 1fr);
    grid-template-rows: minmax(0, 1fr) auto;
    align-items: center;
    gap: 8px;
    height: 100%;
    padding: 8px;
    min-width: 0;
    container-type: size;
  }
  .step {
    height: 100%;
    min-height: 36px;
    font-size: var(--fs-xl);
    padding: 0;
  }
  .readout {
    display: grid;
    justify-items: center;
    gap: 2px;
    min-width: 0;
  }
  .value {
    font-size: clamp(16px, 40cqh, 28px);
    font-weight: 600;
    color: var(--text-strong);
    white-space: nowrap;
  }
  .value small {
    margin-left: 4px;
    font-size: 0.5em;
    font-weight: 500;
    color: var(--text-faint);
  }
  .robot {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    white-space: nowrap;
  }
  .confirm {
    grid-column: 1 / -1;
    grid-row: 2;
  }
</style>
