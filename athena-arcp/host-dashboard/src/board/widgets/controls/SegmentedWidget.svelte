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
    if (!signal || ctx.editing || value === robot) return;
    pending = value;
    try {
      await ctx.write(signal, value);
    } finally {
      pending = null;
    }
  }
</script>

<div class="segmented">
  <div class="options" role="radiogroup" aria-label={signal?.name ?? 'Choice'}>
    {#each list.options as option (option)}
      {@const active = option === robot}
      <button
        type="button"
        role="radio"
        aria-checked={active}
        class="opt"
        class:active
        class:extra={option === list.extra}
        class:sending={pending === option}
        title={option === list.extra ? 'Robot is set to a value that is not in the list' : option}
        disabled={!signal?.writable || ctx.editing}
        onclick={() => void choose(option)}
      >{option}</button>
    {/each}
  </div>
  <div class="meta">
    {#if list.options.length === 0}<span>No options. Bind a string list or set Choices.</span>{/if}
    <Feedback {reply} {locked} pending={pending !== null} {now} />
  </div>
</div>

<style>
  .segmented {
    display: grid;
    align-content: center;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    min-width: 0;
  }
  .options {
    display: flex;
    flex-wrap: wrap;
    gap: 4px;
    min-width: 0;
  }
  .opt {
    flex: 1 1 auto;
    min-width: 0;
    min-height: 30px;
    padding: 4px 10px;
    border: 1px solid var(--border-subtle);
    border-radius: var(--radius-sm);
    background: var(--surface-3);
    color: var(--text);
    font: inherit;
    font-size: var(--fs-sm);
    cursor: pointer;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .opt:hover:not(:disabled) {
    background: var(--surface-hover);
  }
  .opt.active {
    background: var(--info-soft);
    border-color: var(--info);
    color: var(--text-strong);
    font-weight: 600;
  }
  .opt.extra {
    border-style: dashed;
    color: var(--warn);
  }
  .opt.sending {
    opacity: 0.6;
  }
  .opt:disabled {
    cursor: default;
    opacity: 0.5;
  }
  .meta {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    min-height: 14px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
</style>
