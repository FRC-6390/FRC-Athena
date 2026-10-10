<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';
  import { formatSignalValue, parseInput } from '../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const value = $derived.by(() => {
    void ctx.live.current;
    return ctx.latest(signal);
  });
  const decimals = $derived(ctx.setting<number>('decimals'));
  const text = $derived.by(() => {
    if (!signal) return '--';
    if (typeof value === 'number' && decimals >= 0) return value.toFixed(decimals);
    return formatSignalValue(signal.type, value);
  });
  const unit = $derived(ctx.setting<string>('unit') || signal?.unit || '');
  const finite = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  const warnAbove = $derived(finite(ctx.setting<number>('warnAbove')));
  const faultAbove = $derived(finite(ctx.setting<number>('faultAbove')));
  const tone = $derived(
    typeof value !== 'number'
      ? null
      : faultAbove !== null && value >= faultAbove
        ? 'fault'
        : warnAbove !== null && value >= warnAbove
          ? 'warn'
          : null
  );
  // Scale the number to the widget: big tiles read across the pit, small ones stay compact.
  const fontSize = $derived(Math.max(14, Math.min(ctx.size.height * 0.5, ctx.size.width / Math.max(4, text.length + unit.length * 0.6))));

  let editing = $state(false);
  let draft = $state('');
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = signal ? ctx.reply(signal) : undefined;
    return r && r.op === 'write' ? r : undefined;
  });

  async function commit() {
    if (!signal) return;
    const parsed = parseInput(signal.type, draft);
    editing = false;
    if (parsed !== null) await ctx.write(signal, parsed);
  }
</script>

<div class="value" data-tone={tone}>
  {#if editing}
    <!-- svelte-ignore a11y_autofocus -->
    <input
      class="ui-input"
      autofocus
      bind:value={draft}
      onkeydown={(e) => {
        if (e.key === 'Enter') void commit();
        if (e.key === 'Escape') editing = false;
      }}
      onblur={() => (editing = false)}
    />
  {:else}
    <button
      type="button"
      class="number ui-mono"
      style="font-size:{fontSize}px"
      disabled={!signal?.writable || ctx.editing}
      title={signal?.writable ? 'Click to set' : signal?.path}
      onclick={() => {
        draft = value === undefined ? '' : String(value);
        editing = true;
      }}
    >
      {text}{#if unit}<span class="unit">{unit}</span>{/if}
    </button>
  {/if}
  {#if reply && !reply.accepted}<span class="rejected" title={reply.reason}>rejected: {reply.reason}</span>{/if}
</div>

<style>
  .value {
    display: grid;
    place-items: center;
    height: 100%;
    padding: 4px 10px;
  }
  .number {
    border: none;
    background: transparent;
    color: var(--text-strong);
    font-weight: 600;
    line-height: 1;
    white-space: nowrap;
    cursor: default;
    padding: 0;
  }
  .number:not(:disabled) {
    cursor: pointer;
  }
  .number:disabled {
    opacity: 1;
  }
  .value[data-tone='warn'] .number {
    color: var(--warn);
  }
  .value[data-tone='fault'] .number {
    color: var(--fault);
  }
  .unit {
    margin-left: 0.25em;
    font-size: 0.45em;
    font-weight: 500;
    color: var(--text-faint);
  }
  input {
    width: 100%;
    max-width: 200px;
    text-align: center;
  }
  .rejected {
    font-size: var(--fs-xs);
    color: var(--fault);
  }
</style>
