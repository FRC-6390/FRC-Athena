<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import MechanismsView from './components/mechanisms/MechanismsView.svelte';
  import PitView from './components/mechanisms/PitView.svelte';
  import AnalysisView from './components/analysis/AnalysisView.svelte';
  import DriverView from './components/driver/DriverView.svelte';
  import WorkspaceSwitcher, { type Workspace } from './components/workspaces/WorkspaceSwitcher.svelte';
  import { useLiveStore } from './lib/data/live.svelte';
  import { createMockSource } from './lib/data/mock';
  import { createSession } from './lib/data/session';

  /**
   * Next dashboard shell (F2). Served at `?ui=next`; the classic App remains the default until the
   * workspaces cover the classic canvas. Uses the mock robot until a live source is selected.
   */
  const session = createSession(createMockSource());
  const live = useLiveStore(session.store);

  let workspace = $state<Workspace>('driver');

  onMount(() => session.start());
  onDestroy(() => session.stop());

  const status = $derived.by(() => {
    void live.current;
    const store = session.store;
    return {
      signals: store.allSignals().length,
      markers: store.markers().length,
      gaps: store.sequenceGaps,
      unknown: store.unknownEntries
    };
  });
</script>

<div class="shell">
  <header class="topbar">
    <div class="brand">
      <strong>Athena</strong>
      <span class="source">{session.source.name} source</span>
    </div>
    <WorkspaceSwitcher active={workspace} onselect={(next) => (workspace = next)} />
    <div class="status">
      <span>{status.signals} signals</span>
      <span>{status.markers} markers</span>
      {#if status.gaps > 0}<span class="warn">{status.gaps} frame gaps</span>{/if}
      <a href="?ui=classic">Classic dashboard</a>
    </div>
  </header>

  <main>
    {#if workspace === 'driver'}
      <DriverView {session} {live} />
    {:else if workspace === 'pit'}
      <section class="panel">
        <h2>Pit</h2>
        <PitView {session} {live} />
      </section>
    {:else if workspace === 'tuning'}
      <section class="panel">
        <h2>Tuning</h2>
        <MechanismsView {session} {live} />
      </section>
    {:else}
      <section class="panel">
        <h2>Analysis</h2>
        <AnalysisView />
      </section>
    {/if}
  </main>
</div>

<style>
  .shell {
    height: 100%;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    color: var(--text);
  }
  .topbar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 12px;
    padding: 10px 16px;
    border-bottom: 1px solid var(--border);
    background: rgba(15, 17, 23, 0.7);
  }
  .brand {
    display: flex;
    gap: 10px;
    align-items: baseline;
    font-family: var(--font-display);
  }
  .source {
    font-size: 12px;
    color: var(--text-soft);
  }
  .status {
    display: flex;
    gap: 12px;
    align-items: center;
    font-size: 12px;
    color: var(--text-soft);
    flex-wrap: wrap;
  }
  .status .warn {
    color: #fbbf24;
  }
  .status a {
    color: var(--text);
  }
  main {
    overflow: auto;
    padding: 16px;
    min-height: 0;
  }
  .panel {
    display: grid;
    gap: 12px;
    max-width: 1400px;
  }
  h2 {
    margin: 0;
    font-family: var(--font-display);
    font-size: 18px;
    color: var(--text-strong);
  }
  @media (max-width: 720px) {
    main {
      padding: 12px;
    }
  }
</style>
