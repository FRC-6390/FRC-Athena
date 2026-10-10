<script lang="ts">
  import ControlPlot from '../mechanisms/ControlPlot.svelte';
  import { useLiveStore } from '../../lib/data/live.svelte';
  import type { WidgetProps } from '../../lib/widgets/contract';
  import type { PlotWidgetConfig } from '../../lib/widgets/plot';

  /** New-contract `plot` module: a trace of one numeric signal. */
  let { signal, config, store }: WidgetProps<PlotWidgetConfig> = $props();

  // The store is fixed for the lifetime of a session, so capturing it once is intended.
  // svelte-ignore state_referenced_locally
  const live = useLiveStore(store);
</script>

<div class="plot-widget">
  <ControlPlot
    signals={[signal]}
    {store}
    {live}
    windowSeconds={config.windowSeconds}
    yMin={config.yMin}
    yMax={config.yMax}
  />
</div>

<style>
  .plot-widget {
    height: 100%;
    display: grid;
    align-content: start;
    padding: 8px;
  }
</style>
