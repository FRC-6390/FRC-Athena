<script lang="ts">
  import Rail, { type Section } from './Rail.svelte';
  import Header from './Header.svelte';
  import DashboardsScreen from '../screens/DashboardsScreen.svelte';
  import MechanismsScreen from '../screens/MechanismsScreen.svelte';
  import HealthScreen from '../screens/HealthScreen.svelte';
  import SignalsScreen from '../screens/SignalsScreen.svelte';
  import SettingsScreen from '../screens/SettingsScreen.svelte';
  import PitFiles from '../components/pit/PitFiles.svelte';
  import AnalysisView from '../components/analysis/AnalysisView.svelte';
  import ConnectionPanel, { type Connector } from './ConnectionPanel.svelte';
  import { useLiveStore } from '../lib/data/live.svelte';
  import type { Session } from '../lib/data/session';
  import type { LiveSource } from '../lib/data/live-source';
  import { deriveDriverState } from '../lib/driver/driver-state';

  type Props = {
    session: Session;
    /** Robot connection controls; null for the mock robot. */
    connector: Connector | null;
    /** Desktop live source, used by the robot files panel; null elsewhere. */
    liveSource: LiveSource | null;
    sourceName: string;
  };

  let { session, connector, liveSource, sourceName }: Props = $props();
  const useMock = $derived(connector === null);
  const live = useLiveStore(session.store);

  let section = $state<Section>('dashboards');
  let mechanism = $state<string | null>(null);
  let connectionOpen = $state(false);

  const driver = $derived.by(() => {
    void live.current;
    return deriveDriverState(session.store);
  });

  const connection = $derived.by(() => {
    void live.current;
    if (useMock) return { label: 'Mock robot', detail: 'demo data', tone: 'info' as const };
    if (sourceName === 'Bridge' && session.store.allSignals().length === 0)
      return { label: 'Bridge', detail: 'choose a robot', tone: 'stale' as const };
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
        <ConnectionPanel {connector} />
      </div>
    {/if}
    <main class="content" class:flush={section === 'mechanisms' || section === 'dashboards' || section === 'signals'}>
      {#if section === 'dashboards'}
        <DashboardsScreen {session} {live} {driver} onmechanism={openMechanism} />
      {:else if section === 'mechanisms'}
        <MechanismsScreen {session} {live} selected={mechanism} onselect={(p) => (mechanism = p)} />
      {:else if section === 'health'}
        <HealthScreen {session} {live} {driver} onmechanism={openMechanism}>
          {#snippet files()}
            <PitFiles live={liveSource} onopenanalysis={() => (section = 'replays')} />
          {/snippet}
        </HealthScreen>
      {:else if section === 'replays'}
        <section class="ui-panel"><div class="ui-panel-body"><AnalysisView /></div></section>
      {:else if section === 'signals'}
        <SignalsScreen {session} {live} />
      {:else}
        <SettingsScreen {session} {sourceName} />
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
  .connection-popover {
    position: absolute;
    top: calc(var(--topbar-height) + 6px);
    right: var(--sp-4);
    z-index: 20;
    width: min(560px, calc(100% - 32px));
    padding: var(--sp-3);
    box-shadow: 0 12px 32px rgba(0, 0, 0, 0.45);
  }
</style>
