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
  const finite = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  // Without explicit bounds the range follows the robot value so a gain can't be flung to 100 by accident.
  let anchor = $state<number | null>(null);
  $effect(() => {
    if (anchor === null && signal && ctx.latest(signal) !== undefined) anchor = Math.abs(robotValue);
  });
  const autoMax = $derived(Math.max(1, (anchor ?? 0) * 4));
  const min = $derived(finite(ctx.setting<number>('min')) ?? (robotValue < 0 ? -autoMax : 0));
  const max = $derived(finite(ctx.setting<number>('max')) ?? autoMax);
  const step = $derived(finite(ctx.setting<number>('step')) ?? (max - min) / 200);
  let pending = $state<number | null>(null);
  const live = $derived(ctx.setting<boolean>('sendWhileDragging'));

  let dragging = $state<number | null>(null);
  let lastSent = 0;
  const shown = $derived(dragging ?? pending ?? robotValue);
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
    if (signal && dragging !== null) {
      const big = Math.abs(dragging - robotValue) > Math.max(Math.abs(robotValue) * 0.5, step);
      if (big && !live) pending = dragging;
      else void ctx.write(signal, dragging);
    }
    dragging = null;
  }
  function confirm() {
    if (signal && pending !== null) void ctx.write(signal, pending);
    pending = null;
  }
</script>

<div class="slider">
  <div class="top">
    <span class="v ui-mono">{formatNumber(shown)}{#if signal?.unit}<small>{signal.unit}</small>{/if}</span>
    {#if (dragging ?? pending) !== null && Math.abs((dragging ?? pending ?? 0) - robotValue) > step / 2}
      <span class="robot ui-mono">robot {formatNumber(robotValue)}</span>
    {/if}
  </div>
  <input type="range" {min} {max} {step} value={shown} disabled={ctx.editing || !signal} oninput={input} onchange={change} />
  {#if pending !== null}
    <div class="confirm">
      <span>Send {formatNumber(pending)}? (robot {formatNumber(robotValue)})</span>
      <button type="button" class="ui-btn" data-size="sm" data-variant="primary" onclick={confirm}>Send</button>
      <button type="button" class="ui-btn" data-size="sm" onclick={() => (pending = null)}>Cancel</button>
    </div>
  {/if}
  <div class="bounds ui-mono"><span>{formatNumber(min)}</span>{#if reply && !reply.accepted}<span class="rejected">{reply.reason}</span>{/if}<span>{formatNumber(max)}</span></div>
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
  .confirm {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: var(--fs-xs);
    color: var(--warn);
  }
  .confirm span {
    flex: 1;
  }
</style>
