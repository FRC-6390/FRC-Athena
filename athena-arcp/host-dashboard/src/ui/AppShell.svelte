<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import Rail, { type Section } from './Rail.svelte';
  import Header from './Header.svelte';
  import DashboardsScreen from '../screens/DashboardsScreen.svelte';
  import MechanismsScreen from '../screens/MechanismsScreen.svelte';
  import PitView from '../components/mechanisms/PitView.svelte';
  import PitFiles from '../components/pit/PitFiles.svelte';
  import AnalysisView from '../components/analysis/AnalysisView.svelte';
  import ConnectionBar from '../components/live/ConnectionBar.svelte';
  import { useLiveStore } from '../lib/data/live.svelte';
  import { createMockSource } from '../lib/data/mock';
  import { createSession } from '../lib/data/session';
  import { isTauriRuntime, LiveSource } from '../lib/data/live-source';
  import { deriveDriverState } from '../lib/driver/driver-state';

  const useMock = new URLSearchParams(window.location.search).get('source') === 'mock' || !isTauriRuntime();
  const liveSource = useMock ? null : new LiveSource();
  const session = createSession(liveSource ?? createMockSource());
  const live = useLiveStore(session.store);

  let section = $state<Section>('dashboards');
  let mechanism = $state<string | null>(null);
  let connectionOpen = $state(false);

  onMount(() => session.start());
  onDestroy(() => session.stop());

  const driver = $derived.by(() => {
    void live.current;
    return deriveDriverState(session.store);
  });

  const connection = $derived.by(() => {
    void live.current;
    if (useMock) return { label: 'Mock robot', detail: 'demo data', tone: 'info' as const };
    const signals = session.store.allSignals().length;
    return signals > 0
      ? { label: 'Connected', detail: `${signals} signals`, tone: 'ok' as const }
      : { label: 'Not connected', detail: 'choose a robot', tone: 'stale' as const };
  });

  const titles: Record<Section, string> = {
    dashboards: 'Dashboards',
    mechanisms: 'Mechanisms',
    health: 'Health',
    replays: 'Replays',
    signals: 'Signals',
    settings: 'Settings'
  };

  const attention = $derived.by(() => {
    const faults = driver.alerts.some((a) => a.level === 'fault');
    const warns = driver.alerts.length > 0;
    return faults ? { health: 'fault' as const } : warns ? { health: 'warn' as const } : {};
  });

  const signalRows = $derived.by(() => {
    void live.current;
    if (section !== 'signals') return [];
    return session.store
      .allSignals()
      .slice()
      .sort((a, b) => a.path.localeCompare(b.path))
      .map((s) => ({ s, value: session.store.latestSample(s.signalId)?.value }));
  });
  let signalFilter = $state('');

  function openMechanism(path: string) {
    mechanism = path;
    section = 'mechanisms';
  }
</script>

<div class="app">
  <Rail {section} onselect={(next) => (section = next)} {attention} />
  <div class="main">
    <Header
      title={titles[section]}
      subtitle={section === 'mechanisms' && mechanism ? mechanism : undefined}
      {connection}
      {driver}
      onconnection={() => (connectionOpen = !connectionOpen)}
    />
    {#if connectionOpen}
      <div class="connection-popover ui-panel">
        {#if liveSource}
          <ConnectionBar live={liveSource} />
        {:else}
          <p class="note">
            Showing demo data. Open the desktop app (or run the ARCP bridge) to connect to a robot or the simulator.
          </p>
        {/if}
      </div>
    {/if}
    <main class="content" class:flush={section === 'mechanisms' || section === 'dashboards'}>
      {#if section === 'dashboards'}
        <DashboardsScreen {session} {live} {driver} onmechanism={openMechanism} />
      {:else if section === 'mechanisms'}
        <MechanismsScreen {session} {live} selected={mechanism} onselect={(p) => (mechanism = p)} />
      {:else if section === 'health'}
        <div class="stack">
          <section class="ui-panel"><header class="ui-panel-head"><h3 class="ui-panel-title">Robot health</h3></header><div class="ui-panel-body"><PitView {session} {live} /></div></section>
          <section class="ui-panel"><header class="ui-panel-head"><h3 class="ui-panel-title">Robot files</h3></header><div class="ui-panel-body"><PitFiles live={liveSource} onopenanalysis={() => (section = 'replays')} /></div></section>
        </div>
      {:else if section === 'replays'}
        <section class="ui-panel"><div class="ui-panel-body"><AnalysisView /></div></section>
      {:else if section === 'signals'}
        <section class="ui-panel signals">
          <div class="search"><input class="ui-input" placeholder="Filter signals" bind:value={signalFilter} aria-label="Filter signals" /></div>
          <div class="table">
            {#each signalRows.filter((r) => r.s.path.includes(signalFilter)) as row (row.s.signalId)}
              <div class="ui-kv">
                <span class="ui-kv-key ui-mono">{row.s.path}</span>
                <span class="ui-kv-value">{row.value === undefined ? '—' : String(row.value)}</span>
              </div>
            {/each}
          </div>
        </section>
      {:else}
        <section class="ui-panel"><div class="ui-panel-body">Settings</div></section>
      {/if}
    </main>
  </div>
</div>

<style>
  .app {
    display: grid;
    grid-template-columns: var(--rail-width) minmax(0, 1fr);
    height: 100vh;
    overflow: hidden;
  }
  .main {
    position: relative;
    display: grid;
    grid-template-rows: var(--topbar-height) minmax(0, 1fr);
    min-width: 0;
    min-height: 0;
  }
  .content {
    padding: var(--sp-4);
    overflow: auto;
    min-height: 0;
  }
  .content.flush {
    overflow: hidden;
  }
  .stack {
    display: grid;
    gap: var(--sp-3);
  }
  .connection-popover {
    position: absolute;
    top: calc(var(--topbar-height) + 6px);
    right: var(--sp-4);
    z-index: 20;
    width: min(560px, calc(100% - 32px));
    padding: var(--sp-3);
    box-shadow: 0 12px 32px rgba(0, 0, 0, 0.45);
  }
  .note {
    margin: 0;
    font-size: var(--fs-sm);
    color: var(--text-soft);
  }
  .signals {
    display: flex;
    flex-direction: column;
    max-height: 100%;
  }
  .signals .search {
    padding: var(--sp-2);
    border-bottom: 1px solid var(--border);
  }
  .signals .search input {
    width: 100%;
    font-family: var(--font-body);
  }
  .table {
    overflow: auto;
  }
</style>
