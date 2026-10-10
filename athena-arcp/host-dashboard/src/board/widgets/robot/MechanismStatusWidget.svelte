<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { buildMechanismTree, flattenMechanisms } from '../../../lib/data/tree';
  import { summarize } from '../../../lib/mechanisms/model';
  import { humanLabel } from '../../../lib/board/widgets/controls-values';
  import { reasonTone } from '../../../screens/mechanisms/tones';

  let { ctx }: { ctx: WidgetContext } = $props();

  /** Bound mechanisms (any signal of each), or every mechanism when nothing is bound. */
  const rows = $derived.by(() => {
    void ctx.live.current;
    const store = ctx.session.store;
    const roots = buildMechanismTree(store.allSignals());
    const robot = roots.find((r) => r.name === 'robot');
    const all = flattenMechanisms(robot ? robot.children : roots).filter((n) => n.signals.length > 0);
    const bound = new Set((ctx.bindings.mechanisms ?? []).map((s) => s.mechanismPath));
    const text = (id: number | undefined) => {
      if (id === undefined) return '';
      const v = store.latestSample(id)?.value;
      return typeof v === 'string' ? v : '';
    };
    return all
      .filter((n) => bound.size === 0 || bound.has(n.path))
      .map((node) => {
        const s = summarize(node);
        const fault = text(s.fault?.signalId);
        const action = text(s.state?.signalId);
        const reason = text(s.reason?.signalId);
        const tone = fault ? 'fault' : action ? 'info' : reason ? reasonTone(reason) : 'stale';
        return { path: node.path, name: humanLabel(node.name), fault, action, reason, tone };
      });
  });

  const hideIdle = $derived(ctx.setting<boolean>('hideIdle') === true);
  const shown = $derived(hideIdle ? rows.filter((r) => r.fault || r.action) : rows);
  const faults = $derived(rows.filter((r) => r.fault).length);
</script>

<div class="status">
  {#if shown.length === 0}
    <div class="empty">{rows.length === 0 ? 'No mechanisms' : 'All mechanisms idle'}</div>
  {:else}
    <ul style="--min:{ctx.size.width < 320 ? '100%' : '150px'}">
      {#each shown as row (row.path)}
        <li data-tone={row.tone} title={[row.path, row.fault, row.reason].filter(Boolean).join('\n')}>
          <span class="ui-dot" data-tone={row.tone}></span>
          <span class="name">{row.name}</span>
          <span class="detail ui-mono">{row.fault || row.action || row.reason || '--'}</span>
        </li>
      {/each}
    </ul>
  {/if}
  {#if faults > 0}<div class="summary" data-tone="fault">{faults} faulted</div>{/if}
</div>

<style>
  .status {
    display: flex;
    flex-direction: column;
    height: 100%;
    min-height: 0;
    padding: 6px;
    box-sizing: border-box;
    gap: 6px;
  }
  ul {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(var(--min), 1fr));
    gap: 4px;
    margin: 0;
    padding: 0;
    list-style: none;
    overflow: auto;
    min-height: 0;
    align-content: start;
  }
  li {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr);
    grid-template-rows: auto auto;
    column-gap: 6px;
    align-items: center;
    padding: 5px 8px;
    border-radius: var(--radius-sm);
    background: var(--surface-2);
    border: 1px solid transparent;
  }
  li[data-tone='fault'] {
    border-color: var(--fault);
    background: var(--fault-soft);
  }
  li[data-tone='info'] {
    border-color: color-mix(in srgb, var(--info) 45%, transparent);
  }
  .name {
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .detail {
    grid-column: 2;
    font-size: var(--fs-xs);
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  li[data-tone='fault'] .detail {
    color: var(--fault);
  }
  .empty {
    display: grid;
    place-items: center;
    flex: 1;
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
  .summary {
    font-size: var(--fs-xs);
    font-weight: 600;
    color: var(--fault);
  }
</style>
