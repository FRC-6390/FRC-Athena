<script lang="ts">
  import { getContext } from 'svelte';
  import { addGraphSeries, readGraphConfig } from '../../lib/widget-config';
  import type { GraphSeriesStyle } from '../../lib/widget-config';
  import {
    isNumeric,
    parseOptionalNumber,
    withSeriesColor,
    withSeriesStyle,
    withoutSeries
  } from '../../lib/widget-config/editor-helpers';
  import type { SignalSectionProps, SignalMapRequest } from './types';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const requestSignalMap =
    getContext<((request: SignalMapRequest) => void) | undefined>('arcp.signal-map-request');

  function requestGraphSignalFromExplorer() {
    if (!primarySignal) return;
    if (!requestSignalMap) {
      addSelectedSignalToGraph();
      return;
    }
    requestSignalMap({
      title: 'Add graph series',
      candidates: numericSignals,
      selectedSignalId: isNumeric(selectedSignal) ? selectedSignal.signal_id : null,
      allowNone: false,
      noneLabel: 'Required',
      onPick: (signalId) => {
        if (signalId === null) return;
        const signal = signalById.get(signalId);
        if (!isNumeric(signal)) return;
        setConfig(addGraphSeries(selectedWidget.config, primarySignal, signal));
      }
    });
  }
  function updateGraphSeriesStyle(signalId: number, style: GraphSeriesStyle) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    setConfig(withSeriesStyle(graph, signalId, style));
  }
  function updateGraphSeriesColor(signalId: number, color: string) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    setConfig(withSeriesColor(graph, signalId, color));
  }
  function removeGraphSeries(signalId: number) {
    if (!primarySignal) return;
    const graph = readGraphConfig(selectedWidget.config, primarySignal);
    const next = withoutSeries(graph, signalId);
    if (next) setConfig(next);
  }
  function addSelectedSignalToGraph() {
    if (!primarySignal || !isNumeric(selectedSignal)) return;
    setConfig(addGraphSeries(selectedWidget.config, primarySignal, selectedSignal));
  }
</script>

{#if selectedWidget.kind === 'graph'}
  {@const graph = readGraphConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Graph</h4>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={graph.showLegend}
        onchange={(event) => {
          setConfig({ ...graph, showLegend: (event.currentTarget as HTMLInputElement).checked });
        }}
      />
      Show legend
    </label>

    <div class="row-grid">
      <label>
        Y Min
        <input
          type="number"
          value={graph.yMin ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value;
            setConfig({ ...graph, yMin: parseOptionalNumber(raw) });
          }}
        />
      </label>
      <label>
        Y Max
        <input
          type="number"
          value={graph.yMax ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value;
            setConfig({ ...graph, yMax: parseOptionalNumber(raw) });
          }}
        />
      </label>
    </div>

    <button class="btn" disabled={numericSignals.length === 0} onclick={requestGraphSignalFromExplorer}>
      Add signal from explorer
    </button>

    <div class="series-list">
      {#each graph.series as series (series.signalId)}
        {@const bound = signalById.get(series.signalId)}
        <div class="series-row">
          <strong>{bound?.path ?? `Signal ${series.signalId}`}</strong>
          <select
            value={series.style}
            onchange={(event) =>
              updateGraphSeriesStyle(
                series.signalId,
                (event.currentTarget as HTMLSelectElement).value as GraphSeriesStyle
              )}
          >
            <option value="line">Line</option>
            <option value="step">Step</option>
            <option value="dot">Dot</option>
          </select>
          <input
            type="color"
            value={series.color}
            oninput={(event) =>
              updateGraphSeriesColor(series.signalId, (event.currentTarget as HTMLInputElement).value)}
          />
          <button class="btn btn-danger" disabled={graph.series.length <= 1} onclick={() => removeGraphSeries(series.signalId)}>
            Remove
          </button>
        </div>
      {/each}
    </div>
  </section>
{/if}
