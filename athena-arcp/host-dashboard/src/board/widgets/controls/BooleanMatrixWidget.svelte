<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { humanLabel, matrixColumns } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const bits = $derived(ctx.bindings.bits ?? []);
  const columns = $derived(matrixColumns(bits.length, ctx.setting<number>('columns')));
  const trueTone = $derived(ctx.setting<string>('trueTone') || 'ok');
  const falseTone = $derived(ctx.setting<string>('falseTone') || 'off');
  const states = $derived.by(() => {
    void ctx.live.current;
    return bits.map((s) => {
      const v = ctx.latest(s);
      return { signal: s, tone: v === undefined ? 'stale' : v === true ? trueTone : falseTone, on: v === true };
    });
  });
</script>

<div class="matrix" style="grid-template-columns: repeat({columns}, minmax(0, 1fr))">
  {#each states as cell (cell.signal.path)}
    <div class="cell" data-tone={cell.tone} title={cell.signal.path}>
      <span class="light"></span>
      <span class="label">{humanLabel(cell.signal.name)}</span>
    </div>
  {/each}
  {#if bits.length === 0}
    <div class="empty">Bind boolean signals</div>
  {/if}
</div>

<style>
  .matrix {
    display: grid;
    gap: 4px;
    align-content: start;
    height: 100%;
    padding: 6px;
    overflow: auto;
  }
  .cell {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
    min-height: 26px;
    padding: 4px 8px;
    border: 1px solid var(--border);
    border-radius: var(--radius-sm);
    background: var(--surface-2);
  }
  .light {
    flex: none;
    width: 12px;
    height: 12px;
    border-radius: 50%;
    background: var(--surface-3);
    border: 1px solid var(--border-emphasis);
  }
  .cell[data-tone='ok'] .light {
    background: var(--ok);
    border-color: var(--ok);
    box-shadow: 0 0 8px var(--ok-soft);
  }
  .cell[data-tone='warn'] .light {
    background: var(--warn);
    border-color: var(--warn);
  }
  .cell[data-tone='fault'] .light {
    background: var(--fault);
    border-color: var(--fault);
  }
  .cell[data-tone='info'] .light {
    background: var(--info);
    border-color: var(--info);
  }
  .cell[data-tone='stale'] .light {
    background: transparent;
    border-style: dashed;
  }
  .label {
    min-width: 0;
    font-size: var(--fs-xs);
    color: var(--text);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .cell[data-tone='stale'] .label {
    color: var(--text-faint);
  }
  .empty {
    display: grid;
    place-items: center;
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
