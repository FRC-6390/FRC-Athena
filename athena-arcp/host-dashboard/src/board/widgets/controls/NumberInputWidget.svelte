<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { confirmReason, describeChange, finiteSetting, nudge, parseNumberInput } from '../../../lib/board/widgets/controls-numeric';
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
  // Only user-set limits clamp the buttons; with none set, steps are unbounded (the confirm guard still applies).
  const bounds = $derived.by(() => {
    const lo = finiteSetting(userMin);
    const hi = finiteSetting(userMax);
    return lo === null && hi === null ? undefined : { min: lo ?? -Infinity, max: hi ?? Infinity };
  });
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

  // The draft follows the robot until the user types; `null` means "show the robot value".
  let draft = $state<string | null>(null);
  let pending = $state<number | null>(null);
  let error = $state('');
  let confirm = $state<{ value: number; reason: string } | null>(null);

  const shown = $derived(draft ?? (robot === null ? '' : formatNumber(robot)));
  const typed = $derived(parseNumberInput(draft));
  const differs = $derived(draft !== null && typed !== null && typed !== robot);

  function reasonText(value: number, reason: string): string {
    if (reason === 'range') return `Send ${value}? It is outside the limits ${userMin ?? ''} to ${userMax ?? ''}.`;
    if (reason === 'unknown') return `Send ${value}? The robot value is unknown, so the size of the change cannot be checked.`;
    return `Send ${describeChange(robot, value)}? That is a large change.`;
  }

  async function send(value: number) {
    if (!signal) return;
    pending = value;
    try {
      await ctx.write(signal, value);
    } finally {
      pending = null;
      draft = null;
      confirm = null;
    }
  }

  function submit() {
    if (!signal || ctx.editing) return;
    const value = parseNumberInput(draft);
    if (value === null) {
      error = 'Enter a number';
      return;
    }
    error = '';
    const reason = confirmReason(robot, value, { userMin, userMax });
    if (reason) confirm = { value, reason };
    else void send(value);
  }

  function step1(direction: 1 | -1) {
    const current = typed ?? robot ?? 0;
    draft = String(nudge(current, direction, step, bounds));
    error = '';
  }
</script>

<div class="number-input">
  <div class="row">
    <button type="button" class="ui-btn step" aria-label="Decrease" disabled={!signal?.writable || ctx.editing} onclick={() => step1(-1)}>−</button>
    <input
      class="ui-input field ui-mono"
      type="text"
      inputmode="decimal"
      aria-label="Value to send"
      placeholder={robot === null ? '--' : ''}
      value={shown}
      disabled={!signal?.writable || ctx.editing}
      oninput={(e) => {
        draft = e.currentTarget.value;
        confirm = null;
      }}
      onkeydown={(e) => {
        if (e.key === 'Enter') submit();
        if (e.key === 'Escape') {
          draft = null;
          error = '';
          confirm = null;
        }
      }}
    />
    <button type="button" class="ui-btn step" aria-label="Increase" disabled={!signal?.writable || ctx.editing} onclick={() => step1(1)}>+</button>
    <button type="button" class="ui-btn send" data-variant="primary" disabled={!signal?.writable || ctx.editing || pending !== null} onclick={submit}>Send</button>
  </div>
  <div class="meta ui-mono">
    <span class="robot">robot {robot === null ? '--' : formatNumber(robot)}{#if signal?.unit} {signal.unit}{/if}</span>
    {#if differs}<span class="draft">draft {typed}</span>{/if}
    {#if error}<span class="error">{error}</span>{/if}
  </div>
  {#if confirm}
    <ConfirmBar text={reasonText(confirm.value, confirm.reason)} onconfirm={() => confirm && send(confirm.value)} oncancel={() => (confirm = null)} />
  {/if}
  <div class="status">
    <Feedback {reply} {locked} pending={pending !== null} {now} />
  </div>
</div>

<style>
  .number-input {
    display: grid;
    align-content: start;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    min-height: 0;
    overflow: auto;
    container-type: inline-size;
  }
  .row {
    display: flex;
    gap: 6px;
    align-items: center;
    min-width: 0;
  }
  .step {
    width: 32px;
    padding: 0;
    font-size: var(--fs-lg);
    flex: none;
  }
  .field {
    flex: 1;
    min-width: 0;
    height: 32px;
    font-size: clamp(14px, 6cqi, 22px);
    text-align: right;
  }
  .send {
    flex: none;
    height: 32px;
  }
  .meta {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .draft {
    color: var(--info);
  }
  .error {
    color: var(--fault);
  }
  .status {
    min-height: 16px;
  }
</style>
