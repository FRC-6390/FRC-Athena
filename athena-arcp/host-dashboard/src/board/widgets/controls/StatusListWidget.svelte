<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatSignalValue } from '../../../lib/format';
  import { humanLabel, rowTone } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const rows = $derived(ctx.bindings.rows ?? []);
  const rules = $derived({
    warnAbove: ctx.setting<number>('warnAbove'),
    faultAbove: ctx.setting<number>('faultAbove'),
    warnBelow: ctx.setting<number>('warnBelow'),
    faultBelow: ctx.setting<number>('faultBelow'),
    trueTone: ctx.setting<string>('trueTone') || 'ok',
    falseTone: ctx.setting<string>('falseTone') || 'off'
  });
  const items = $derived.by(() => {
    void ctx.live.current;
    return rows.map((s) => {
      const v = ctx.latest(s);
      return { signal: s, tone: rowTone(v, rules), text: v === undefined ? '--' : formatSignalValue(s.type, v), unit: s.unit };
    });
  });
</script>

<div class="list">
  {#each items as row (row.signal.path)}
    <div class="row" title={row.signal.path}>
      <span class="dot" data-tone={row.tone ?? 'none'}></span>
      <span class="name">{humanLabel(row.signal.name)}</span>
      <span class="value ui-mono">{row.text}{#if row.unit && row.text !== '--'}<small>{row.unit}</small>{/if}</span>
    </div>
  {/each}
  {#if rows.length === 0}
    <div class="empty">Bind signals to list</div>
  {/if}
</div>

<style>
  .list {
    display: flex;
    flex-direction: column;
    height: 100%;
    overflow: auto;
    min-width: 0;
  }
  .row {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 8px;
    min-height: 26px;
    padding: 0 10px;
    border-top: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  .row:first-child {
    border-top: none;
  }
  .dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--surface-3);
  }
  .dot[data-tone='ok'] {
    background: var(--ok);
  }
  .dot[data-tone='warn'] {
    background: var(--warn);
  }
  .dot[data-tone='fault'] {
    background: var(--fault);
  }
  .dot[data-tone='info'] {
    background: var(--info);
  }
  .dot[data-tone='stale'] {
    background: var(--stale);
  }
  .name {
    min-width: 0;
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .value {
    color: var(--text-strong);
    white-space: nowrap;
  }
  .value small {
    margin-left: 3px;
    font-size: 0.85em;
    color: var(--text-faint);
  }
  .empty {
    display: grid;
    place-items: center;
    height: 100%;
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
