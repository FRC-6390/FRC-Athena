<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import Feedback from './parts/Feedback.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const robot = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'string' ? v : null;
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
  const placeholder = $derived(ctx.setting<string>('placeholder') || 'Type and press Enter');

  // Draft follows the robot until the user types. `null` means "show the robot value".
  let draft = $state<string | null>(null);
  let pending = $state<string | null>(null);
  const shown = $derived(draft ?? robot ?? '');

  async function send() {
    if (!signal || ctx.editing || draft === null) return;
    const value = draft;
    pending = value;
    try {
      await ctx.write(signal, value);
    } finally {
      pending = null;
      draft = null;
    }
  }
</script>

<div class="text-input">
  <div class="row">
    <input
      class="ui-input field"
      type="text"
      aria-label="Text to send"
      {placeholder}
      value={shown}
      disabled={!signal?.writable || ctx.editing}
      oninput={(e) => (draft = e.currentTarget.value)}
      onkeydown={(e) => {
        if (e.key === 'Enter') void send();
        if (e.key === 'Escape') draft = null;
      }}
    />
    <button type="button" class="ui-btn" data-variant="primary" disabled={!signal?.writable || ctx.editing || draft === null || pending !== null} onclick={send}>Send</button>
  </div>
  <div class="meta">
    <span class="robot ui-mono" title={robot ?? ''}>robot: {robot === null ? '--' : robot === '' ? '""' : robot}</span>
    <Feedback {reply} {locked} pending={pending !== null} {now} />
  </div>
</div>

<style>
  .text-input {
    display: grid;
    align-content: center;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    min-width: 0;
  }
  .row {
    display: flex;
    gap: 6px;
    min-width: 0;
  }
  .field {
    flex: 1;
    min-width: 0;
    height: 30px;
  }
  .field:disabled {
    opacity: 0.55;
  }
  .meta {
    display: flex;
    gap: 10px;
    align-items: center;
    min-width: 0;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .robot {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
