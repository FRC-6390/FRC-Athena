<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { choiceList, resolveChoices } from '../../../lib/board/widgets/controls-choice';
  import Feedback from './parts/Feedback.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const optionSignal = $derived(ctx.bindings.options?.[0]);
  const robot = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'string' ? v : undefined;
  });
  const options = $derived.by(() => {
    void ctx.live.current;
    return resolveChoices(ctx.latest(optionSignal), ctx.setting<string>('choices'));
  });
  const list = $derived(choiceList(options, robot));
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

  let pending = $state<string | null>(null);

  async function choose(value: string) {
    if (!signal || ctx.editing) return;
    pending = value;
    try {
      await ctx.write(signal, value);
    } finally {
      pending = null;
    }
  }
</script>

<div class="dropdown">
  <select
    class="ui-input select"
    aria-label={signal?.name ?? 'Choice'}
    value={robot ?? ''}
    disabled={!signal?.writable || ctx.editing || list.options.length === 0}
    onchange={(e) => void choose(e.currentTarget.value)}
  >
    {#if robot === undefined}<option value="" disabled>--</option>{/if}
    {#each list.options as option (option)}
      <option value={option}>{option}{option === list.extra ? ' (robot)' : ''}</option>
    {/each}
  </select>
  <div class="meta">
    {#if list.options.length === 0}<span class="hint">No options. Bind a string list or set Choices.</span>{/if}
    <Feedback {reply} {locked} pending={pending !== null} {now} />
  </div>
</div>

<style>
  .dropdown {
    display: grid;
    align-content: center;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    min-width: 0;
  }
  .select {
    width: 100%;
    height: 32px;
    font-family: var(--font-body);
    font-size: var(--fs-md);
  }
  .meta {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    min-height: 14px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .hint {
    color: var(--text-faint);
  }
</style>
