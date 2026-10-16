<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { ControlSection } from '../../lib/mechanisms/model';
  import TimeChart from '../../ui/TimeChart.svelte';
  import type { ChartSeries } from '../../lib/chart/time-chart';
  import ValueRow from '../../ui/ValueRow.svelte';

  type Props = {
    section: ControlSection;
    session: Session;
    live: { readonly current: number };
  };

  let { section, session, live }: Props = $props();
  let showOther = $state(false);

  const PLOT_COLORS: Record<string, string> = {
    target: 'var(--series-target)',
    reference: 'var(--series-reference)',
    measured: 'var(--series-measured)'
  };
  const PLOT_FALLBACK = ['var(--series-4)', 'var(--series-5)', 'var(--text)'];

  const plotLines = $derived.by((): ChartSeries[] => {
    void live.current;
    return section.plot.map((signal, i) => ({
      id: String(signal.signalId),
      label: signal.name,
      color: PLOT_COLORS[signal.name] ?? PLOT_FALLBACK[i % PLOT_FALLBACK.length],
      samples: session.store.samples(signal.signalId)
    }));
  });

  const constraintLabel = (name: string) =>
    name.replace(/^max/, 'max ').replace(/^min/, 'min ').replace(/([a-z])([A-Z])/g, '$1 $2').toLowerCase();
</script>

<section class="ui-panel card">
  <header class="ui-panel-head">
    <h3 class="ui-panel-title">Control · {section.name}</h3>
    <span class="ui-panel-sub">{section.path}</span>
  </header>
  {#if section.plot.length > 0}
    <div class="ui-panel-body">
      <TimeChart series={plotLines} windowSeconds={10} height={170} />
    </div>
  {/if}
  <div class="tables">
    {#if section.gains.length > 0}
      <div class="table">
        <div class="ui-label head">Gains</div>
        {#each section.gains as signal (signal.signalId)}
          <ValueRow {signal} {session} {live} />
        {/each}
      </div>
    {/if}
    {#if section.constraints.length > 0}
      <div class="table">
        <div class="ui-label head">Constraints</div>
        {#each section.constraints as signal (signal.signalId)}
          <ValueRow {signal} {session} {live} label={constraintLabel(signal.name)} />
        {/each}
      </div>
    {/if}
  </div>
  {#if section.other.length > 0}
    <button type="button" class="more" onclick={() => (showOther = !showOther)}>
      {showOther ? 'Hide' : 'Show'} {section.other.length} more settings
    </button>
    {#if showOther}
      {#each section.other as signal (signal.signalId)}
        <ValueRow {signal} {session} {live} label={signal.localPath.split('/').slice(2).join('/') || signal.name} />
      {/each}
    {/if}
  {/if}
</section>

<style>
  .card {
    display: flex;
    flex-direction: column;
  }
  .tables {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
    border-top: 1px solid var(--border);
  }
  .table + .table {
    border-left: 1px solid var(--border);
  }
  .more {
    padding: 8px var(--sp-3);
    border: none;
    border-top: 1px solid var(--border);
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-xs);
    text-align: left;
    cursor: pointer;
  }
  .more:hover {
    color: var(--text);
  }
  .head {
    padding: var(--sp-2) var(--sp-3) 6px;
  }
</style>
