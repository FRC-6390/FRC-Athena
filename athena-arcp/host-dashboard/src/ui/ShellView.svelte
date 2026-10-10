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
  import { onDestroy, onMount } from 'svelte';
  import ConnectionPanel, { LAST_HOST_STORAGE_KEY, type Connector } from './ConnectionPanel.svelte';
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

  /** Robot link as the source reports it; `unknown` until the first status arrives. */
  let link = $state<'unknown' | 'connected' | 'reconnecting' | 'disconnected'>('unknown');
  const offs: Array<() => void> = [];

  onMount(() => {
    if (!connector) return;
    offs.push(connector.onStatus((status) => {
      if (status.connected) link = 'connected';
      else if (link !== 'reconnecting') link = 'disconnected';
    }));
    if (connector.onEvent) {
      offs.push(connector.onEvent((event) => {
        if (event.kind === 'disconnected') link = event.reconnecting ? 'reconnecting' : 'disconnected';
        if (event.kind === 'reconnected') link = 'connected';
      }));
    }
    // Reconnect to the last robot on launch, unless the source already has a live session.
    let host = '';
    try {
      host = localStorage.getItem(LAST_HOST_STORAGE_KEY) ?? '';
    } catch {
      /* storage unavailable */
    }
    if (host) {
      const timer = setTimeout(() => {
        if (link !== 'connected') void connector.connect({ host }).then(() => (link = 'connected')).catch(() => undefined);
      }, 800);
      offs.push(() => clearTimeout(timer));
    }
  });
  onDestroy(() => offs.forEach((off) => off()));

  const connection = $derived.by(() => {
    void live.current;
    if (useMock) return { label: 'Mock robot', detail: 'demo data', tone: 'info' as const };
    const signals = session.store.allSignals().length;
    if (link === 'reconnecting') return { label: 'Reconnecting', detail: 'robot link lost', tone: 'warn' as const };
    if (link === 'connected') return { label: 'Connected', detail: `${signals} signals`, tone: 'ok' as const };
    return { label: 'Not connected', detail: 'choose a robot', tone: 'stale' as const };
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


  let popover = $state<HTMLDivElement | null>(null);

  /** Escape or a press outside the popover (other than its toggle button) closes it. */
  function closeOnOutside(event: PointerEvent) {
    if (!connectionOpen || !popover) return;
    const target = event.target as Node;
    if (popover.contains(target) || (target as Element).closest?.('[data-connection-toggle]')) return;
    connectionOpen = false;
  }

  function openMechanism(path: string) {
    mechanism = path;
    section = 'mechanisms';
  }
</script>

<svelte:window
  onpointerdown={closeOnOutside}
  onkeydown={(e) => {
    if (e.key === 'Escape') connectionOpen = false;
  }}
/>

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
      <div class="connection-popover ui-panel" bind:this={popover}>
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
