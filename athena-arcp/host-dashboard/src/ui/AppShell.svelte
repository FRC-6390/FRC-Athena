<script lang="ts">
  import { onDestroy, onMount } from 'svelte';
  import ShellView from './ShellView.svelte';
  import type { Connector } from './ConnectionPanel.svelte';
  import { createMockSource } from '../lib/data/mock';
  import { createSession, type Session } from '../lib/data/session';
  import { isTauriRuntime, LiveSource } from '../lib/data/live-source';
  import { WsBridgeSource } from '../lib/data/ws-bridge-source';
  import { DEFAULT_BRIDGE_URL, probeBridge, sourceFromQuery } from '../lib/data/source-select';

  /**
   * Picks the data source, then renders the shell: the desktop app talks to the robot directly; a browser
   * uses `arcp-ws-bridge` when it answers (or `?bridge=ws://...`); `?source=mock` or no bridge shows demo data.
   */
  let session = $state<Session | null>(null);
  let connector = $state<Connector | null>(null);
  let liveSource = $state<LiveSource | null>(null);
  let sourceName = $state('');

  onMount(async () => {
    let choice = sourceFromQuery(window.location.search, isTauriRuntime());
    if (choice === 'probe') {
      choice = (await probeBridge(DEFAULT_BRIDGE_URL)) ? { kind: 'bridge', url: DEFAULT_BRIDGE_URL } : { kind: 'mock' };
    }
    if (choice.kind === 'live') {
      const source = new LiveSource();
      liveSource = source;
      connector = source;
      sourceName = 'Desktop';
      session = createSession(source);
    } else if (choice.kind === 'bridge') {
      const source = new WsBridgeSource({ url: choice.url });
      connector = source;
      sourceName = 'Bridge';
      session = createSession(source);
    } else {
      sourceName = 'Mock robot';
      session = createSession(createMockSource());
    }
    session.start();
  });

  onDestroy(() => session?.stop());
</script>

{#if session}
  <ShellView {session} {connector} {liveSource} {sourceName} />
{/if}
