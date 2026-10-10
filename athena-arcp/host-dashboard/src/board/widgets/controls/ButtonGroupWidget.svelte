<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { humanLabel } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const actions = $derived(ctx.bindings.actions ?? []);
  const columns = $derived.by(() => {
    const n = ctx.setting<number>('columns');
    return typeof n === 'number' && n >= 1 ? Math.floor(n) : Math.max(1, Math.min(actions.length, Math.ceil(Math.sqrt(actions.length) * 1.4)));
  });
  const compact = $derived(ctx.size.width < 160 || ctx.size.height < 80);

  // Paths of actions with a run or cancel in flight.
  let busy = $state<Record<string, true>>({});

  function running(action: SignalDescriptor): boolean {
    void ctx.live.current;
    return ctx.latest(action) === true;
  }
  function reply(action: SignalDescriptor) {
    void ctx.live.current;
    const r = ctx.reply(action);
    return r && (r.op === 'run' || r.op === 'cancel') && !r.accepted ? r : undefined;
  }

  /** Runs an idle action, or stops one that is already running. Every press is sent. */
  async function press(action: SignalDescriptor) {
    if (ctx.editing || busy[action.path]) return;
    busy[action.path] = true;
    try {
      if (running(action)) await ctx.cancel(action);
      else await ctx.run(action);
    } finally {
      delete busy[action.path];
    }
  }
</script>

<div class="group" class:compact style="grid-template-columns: repeat({columns}, minmax(0, 1fr))">
  {#each actions as action (action.path)}
    {@const isRunning = running(action)}
    {@const rejection = reply(action)}
    <button
      type="button"
      class="run"
      class:running={isRunning}
      title={rejection ? `Rejected: ${rejection.reason}` : action.path}
      disabled={ctx.editing}
      onclick={() => void press(action)}
    >
      <span class="name">{humanLabel(action.name)}</span>
      <span class="state">
        {#if busy[action.path]}sending{:else if isRunning}running · stop{:else if rejection}rejected{:else}run{/if}
      </span>
      {#if rejection && rejection.reason}<span class="reason">{rejection.reason}</span>{/if}
    </button>
  {/each}
  {#if actions.length === 0}
    <div class="empty">Bind action signals</div>
  {/if}
</div>

<style>
  .group {
    display: grid;
    gap: 6px;
    height: 100%;
    padding: 6px;
    align-content: stretch;
    grid-auto-rows: minmax(0, 1fr);
  }
  .run {
    display: grid;
    place-content: center;
    gap: 2px;
    min-width: 0;
    min-height: 40px;
    padding: 4px 8px;
    border: 1px solid var(--border-emphasis);
    border-radius: var(--radius);
    background: var(--surface-3);
    color: var(--text-strong);
    font: inherit;
    text-align: center;
    cursor: pointer;
    user-select: none;
  }
  .run:hover:not(:disabled) {
    background: var(--surface-hover);
  }
  .run:active:not(:disabled) {
    transform: translateY(1px);
  }
  .run.running {
    border-color: var(--ok);
    background: var(--ok-soft);
  }
  .run:disabled {
    cursor: default;
    opacity: 0.6;
  }
  .name {
    font-size: var(--fs-md);
    font-weight: 600;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .compact .name {
    font-size: var(--fs-sm);
  }
  .state {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .running .state {
    color: var(--ok);
  }
  .reason {
    font-size: var(--fs-xs);
    color: var(--fault);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .empty {
    display: grid;
    place-items: center;
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
