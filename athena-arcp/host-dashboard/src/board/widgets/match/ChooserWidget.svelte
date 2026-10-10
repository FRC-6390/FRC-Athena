<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { chooserOptions, selectionState } from '../../../lib/board/widgets/match-chooser';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.selected?.[0]);
  const optionSignal = $derived(ctx.bindings.options?.[0]);
  const typed = $derived(ctx.setting<string>('options'));

  const options = $derived.by(() => {
    void ctx.live.current;
    const fromSignal = optionSignal ? ctx.latest(optionSignal) : undefined;
    return chooserOptions(Array.isArray(fromSignal) ? (fromSignal as string[]) : undefined, typed);
  });
  const current = $derived.by(() => {
    void ctx.live.current;
    return signal ? ctx.latest(signal) : undefined;
  });
  const state = $derived(selectionState(options, current));
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = signal ? ctx.reply(signal) : undefined;
    return r && r.op === 'write' ? r : undefined;
  });
  const locked = $derived.by(() => {
    void ctx.live.current;
    return signal ? ctx.locked(signal) : false;
  });

  let pending = $state<string | null>(null);

  async function choose(option: string) {
    if (!signal || ctx.editing || locked) return;
    pending = option;
    try {
      await ctx.write(signal, option);
    } finally {
      pending = null;
    }
  }

  const count = $derived(options.length);
  const columns = $derived(count <= 1 ? 1 : count <= 4 ? 2 : 3);
</script>

<div class="chooser">
  <div class="status">
    <span class="label">Auto</span>
    <span class="chosen" class:none={state.selected === null}>
      {state.selected ?? (state.unknownValue !== null ? `Robot has "${state.unknownValue}"` : 'None chosen')}
    </span>
    {#if locked}<span class="hint">locked</span>{/if}
    {#if reply && !reply.accepted}<span class="hint rejected" title={reply.reason}>{reply.reason}</span>{/if}
  </div>
  {#if options.length === 0}
    <div class="empty">No options. Add them in settings, or bind a string list.</div>
  {:else}
    <div class="grid" style="grid-template-columns: repeat({columns}, minmax(0, 1fr))">
      {#each options as option (option)}
        <button
          type="button"
          class="choice"
          class:selected={state.selected === option}
          class:busy={pending === option}
          disabled={!signal || ctx.editing || locked}
          onclick={() => choose(option)}
        >
          {option}
        </button>
      {/each}
    </div>
  {/if}
  {#if !signal}<div class="hint">Bind a writable string to choose.</div>{/if}
</div>

<style>
  .chooser {
    display: flex;
    flex-direction: column;
    gap: 8px;
    height: 100%;
    padding: 8px;
    box-sizing: border-box;
    container-type: size;
  }
  .status {
    display: flex;
    align-items: baseline;
    flex-wrap: wrap;
    gap: 4px 10px;
  }
  .label {
    font-size: var(--fs-xs);
    font-weight: 700;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--text-soft);
  }
  .chosen {
    font-size: min(9cqh, 7cqw);
    font-weight: 700;
    color: var(--info);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    min-width: 0;
  }
  .chosen.none {
    color: var(--text-faint);
  }
  .grid {
    flex: 1;
    min-height: 0;
    display: grid;
    gap: 6px;
    grid-auto-rows: minmax(40px, 1fr);
  }
  .choice {
    min-width: 0;
    padding: 4px 10px;
    border: 1px solid var(--border-emphasis);
    border-radius: var(--radius);
    background: var(--surface-3);
    color: var(--text-strong);
    font: inherit;
    font-size: min(max(14px, 8cqh), 22px);
    font-weight: 600;
    cursor: pointer;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .choice:hover:not(:disabled) {
    background: var(--surface-hover);
  }
  .choice.selected {
    background: var(--info-soft);
    border-color: var(--info);
    color: var(--info);
    box-shadow: inset 0 0 0 1px var(--info);
  }
  .choice.busy {
    opacity: 0.6;
  }
  .choice:disabled {
    cursor: default;
    opacity: 0.5;
  }
  .choice.selected:disabled {
    opacity: 0.8;
  }
  .empty,
  .hint {
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
  .rejected {
    color: var(--fault);
  }
</style>
