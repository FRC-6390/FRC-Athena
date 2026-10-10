<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { allianceLabel, parseAlliance, parseStation, type Alliance } from '../../../lib/board/widgets/match-alliance';

  let { ctx }: { ctx: WidgetContext } = $props();

  const allianceSignal = $derived(ctx.bindings.alliance?.[0]);
  const stationSignal = $derived(ctx.bindings.station?.[0]);
  const trueMeans = $derived(ctx.setting<Alliance>('trueIs'));

  const alliance = $derived.by(() => {
    void ctx.live.current;
    return allianceSignal ? parseAlliance(ctx.latest(allianceSignal), trueMeans) : null;
  });
  const station = $derived.by(() => {
    void ctx.live.current;
    return stationSignal ? parseStation(ctx.latest(stationSignal)) : null;
  });
  const label = $derived(alliance ? allianceLabel(alliance).toUpperCase() : 'NO ALLIANCE');
</script>

<div class="alliance" data-alliance={alliance ?? 'unknown'}>
  <div class="block">
    <span class="side">{label}</span>
  </div>
  <div class="station">
    <span class="caption">Station</span>
    <span class="number ui-mono">{station ?? '--'}</span>
  </div>
</div>

<style>
  .alliance {
    display: flex;
    flex-direction: row;
    width: 100%;
    height: 100%;
    gap: 6px;
    padding: 6px;
    box-sizing: border-box;
    container-type: size;
  }
  @container (max-aspect-ratio: 1/1) {
    .alliance {
      flex-direction: column;
    }
  }
  .block {
    flex: 3;
    min-width: 0;
    min-height: 0;
    display: grid;
    place-items: center;
    border-radius: var(--radius-lg);
    background: var(--surface-3);
    color: var(--text-soft);
    container-type: size;
  }
  .alliance[data-alliance='red'] .block {
    background: var(--brand);
    color: var(--text-strong);
  }
  .alliance[data-alliance='blue'] .block {
    background: var(--info);
    color: var(--text-strong);
  }
  .side {
    font-size: min(40cqh, 14cqw);
    font-weight: 800;
    letter-spacing: 0.04em;
    line-height: 1.05;
    text-align: center;
    padding: 0 6px;
    overflow-wrap: anywhere;
  }
  .station {
    flex: 2;
    min-width: 0;
    min-height: 0;
    display: grid;
    place-items: center;
    align-content: center;
    gap: 2px;
    border-radius: var(--radius-lg);
    border: 1px solid var(--border);
    background: var(--surface-2);
    container-type: size;
  }
  .caption {
    font-size: min(12cqh, 14cqw);
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--text-soft);
  }
  .number {
    font-size: min(55cqh, 45cqw);
    font-weight: 700;
    line-height: 1;
    color: var(--text-strong);
  }
</style>
