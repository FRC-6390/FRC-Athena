<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';
  import type { ChartSeries } from '../../lib/chart/time-chart';
  import TimeChart from '../../ui/TimeChart.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const PALETTE = ['var(--series-measured)', 'var(--series-reference)', 'var(--series-target)', 'var(--series-4)', 'var(--series-5)', 'var(--text)'];
  const NAMED: Record<string, string> = { target: 'var(--series-target)', reference: 'var(--series-reference)', measured: 'var(--series-measured)' };

  const series = $derived(ctx.bindings.series ?? []);
  const windowSeconds = $derived(ctx.setting<number>('window'));
  const finite = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v : null);
  const fixedMin = $derived(finite(ctx.setting<number>('min')));
  const fixedMax = $derived(finite(ctx.setting<number>('max')));
  const legend = $derived(ctx.setting<boolean>('legend'));

  const lines = $derived.by((): ChartSeries[] => {
    void ctx.live.current;
    return series.map((s, i) => ({
      id: String(s.signalId),
      label: s.name,
      color: NAMED[s.name] ?? PALETTE[i % PALETTE.length],
      samples: ctx.samples(s)
    }));
  });
</script>

<div class="graph">
  <TimeChart series={lines} {windowSeconds} yMin={fixedMin} yMax={fixedMax} legend={legend} />
</div>

<style>
  .graph {
    height: 100%;
    min-height: 0;
    padding: 4px 6px;
    box-sizing: border-box;
  }
</style>
