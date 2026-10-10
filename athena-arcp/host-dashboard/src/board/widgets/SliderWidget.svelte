<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';
  import { formatNumber } from '../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const robotValue = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : 0;
  });
  const min = $derived(ctx.setting<number>('min'));
  const max = $derived(ctx.setting<number>('max'));
  const step = $derived(ctx.setting<number>('step'));
  const live = $derived(ctx.setting<boolean>('sendWhileDragging'));

  let dragging = $state<number | null>(null);
  let lastSent = 0;
  const shown = $derived(dragging ?? robotValue);
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = signal ? ctx.reply(signal) : undefined;
    return r && r.op === 'write' ? r : undefined;
  });

  function input(event: Event) {
    dragging = Number((event.currentTarget as HTMLInputElement).value);
    // Throttle live writes to ~10 Hz so a drag does not flood the robot.
    const now = performance.now();
    if (live && signal && now - lastSent > 100) {
      lastSent = now;
      void ctx.write(signal, dragging);
    }
  }
  function change() {
    if (signal && dragging !== null) void ctx.write(signal, dragging);
    dragging = null;
  }
</script>

<div class="slider">
  <div class="top">
    <span class="v ui-mono">{formatNumber(shown)}{#if signal?.unit}<small>{signal.unit}</small>{/if}</span>
    {#if dragging !== null && Math.abs(dragging - robotValue) > step / 2}
      <span class="robot ui-mono">robot {formatNumber(robotValue)}</span>
    {/if}
  </div>
  <input type="range" {min} {max} {step} value={shown} disabled={ctx.editing || !signal} oninput={input} onchange={change} />
  <div class="bounds ui-mono"><span>{min}</span>{#if reply && !reply.accepted}<span class="rejected">{reply.reason}</span>{/if}<span>{max}</span></div>
</div>

<style>
  .slider {
    display: grid;
    align-content: center;
    gap: 6px;
    height: 100%;
    padding: 6px 12px;
  }
  .top {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
  }
  .v {
    font-size: var(--fs-xl);
    font-weight: 600;
    color: var(--text-strong);
  }
  .v small {
    margin-left: 4px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .robot {
    font-size: var(--fs-xs);
    color: var(--warn);
  }
  input[type='range'] {
    width: 100%;
    accent-color: var(--brand);
  }
  .bounds {
    display: flex;
    justify-content: space-between;
    font-size: 10px;
    color: var(--text-faint);
  }
  .rejected {
    color: var(--fault);
  }
</style>
