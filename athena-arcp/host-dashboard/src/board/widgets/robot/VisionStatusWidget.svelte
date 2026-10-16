<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { humanLabel } from '../../../lib/board/widgets/controls-values';
  import { groupCameras, visionRow, VISION_PREFIX, type VisionRow } from '../../../lib/board/widgets/vision-status';

  let { ctx }: { ctx: WidgetContext } = $props();

  /** Bound cameras (any signal of each), or every camera under athena/vision when nothing is bound. */
  const rows = $derived.by((): { camera: string; row: VisionRow; signals: Record<string, SignalDescriptor> }[] => {
    void ctx.live.current;
    const all = groupCameras(ctx.session.store.allSignals().filter((s) => s.path.startsWith(VISION_PREFIX)));
    const bound = new Set((ctx.bindings.cameras ?? []).map((s) => groupCameras([s])[0]?.name).filter((n): n is string => n !== undefined));
    return all
      .filter((c) => bound.size === 0 || bound.has(c.name))
      .map((c) => ({
        camera: c.name,
        signals: c.signals,
        row: visionRow(c.name, (field) => ctx.latest(c.signals[field]))
      }));
  });
  const warnings = $derived(rows.filter((r) => r.row.tone === 'warn' || r.row.clockNotSynced).length);
</script>

<div class="status">
  {#if rows.length === 0}
    <div class="empty">No vision cameras</div>
  {:else}
    <ul style="--min:{ctx.size.width < 320 ? '100%' : '200px'}">
      {#each rows as { camera, row, signals } (camera)}
        <li data-tone={row.tone} title={[signals.State?.path, row.reason].filter(Boolean).join('\n')}>
          <div class="head">
            <span class="ui-dot" data-tone={row.tone}></span>
            <span class="name">{humanLabel(camera)}</span>
            <span class="ui-badge" data-tone={row.tone}>{row.state}</span>
          </div>
          <div class="detail ui-mono">{row.reason}</div>
          <div class="stats ui-mono">
            <span>acc {row.acceptedPerSecond}/s</span>
            <span>rej {row.rejectedPerSecond}/s</span>
            <span>tags {row.tags}</span>
            <span>frame {row.frameAge}</span>
          </div>
          {#if row.stalled}
            <div class="flag" data-tone="warn">No accepted pose for {row.sinceAccepted}</div>
          {/if}
          {#if row.clockNotSynced}
            <div class="flag" data-tone="warn">Clock not synced</div>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}
  {#if warnings > 0}<div class="summary">{warnings} camera{warnings === 1 ? '' : 's'} need attention</div>{/if}
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
    display: flex;
    flex-direction: column;
    gap: 3px;
    padding: 6px 8px;
    border-radius: var(--radius-sm);
    background: var(--surface-2);
    border: 1px solid transparent;
  }
  li[data-tone='warn'] {
    border-color: var(--warn);
    background: var(--warn-soft);
  }
  li[data-tone='info'] {
    border-color: color-mix(in srgb, var(--info) 45%, transparent);
  }
  .head {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    column-gap: 6px;
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
    font-size: var(--fs-xs);
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    min-height: 1em;
  }
  .stats {
    display: flex;
    flex-wrap: wrap;
    column-gap: 8px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .flag {
    font-size: var(--fs-xs);
    font-weight: 600;
    color: var(--warn);
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
    color: var(--warn);
  }
</style>
