<script lang="ts">
  import type { Session } from '../lib/data/session';

  type Props = {
    session: Session;
    sourceName: string;
  };

  let { session, sourceName }: Props = $props();

  const metadata = $derived(Object.entries(session.store.metadata()));
</script>

<div class="screen">
  <section class="ui-panel">
    <header class="ui-panel-head"><h3 class="ui-panel-title">Session</h3></header>
    <div class="ui-kv"><span class="ui-kv-key">Data source</span><span class="ui-kv-value">{sourceName}</span></div>
    <div class="ui-kv"><span class="ui-kv-key">Signals</span><span class="ui-kv-value">{session.store.allSignals().length}</span></div>
    <div class="ui-kv"><span class="ui-kv-key">Protocol</span><span class="ui-kv-value">ARCP v2</span></div>
  </section>

  <section class="ui-panel">
    <header class="ui-panel-head"><h3 class="ui-panel-title">Robot metadata</h3></header>
    {#each metadata as [key, value] (key)}
      <div class="ui-kv"><span class="ui-kv-key">{key}</span><span class="ui-kv-value">{value}</span></div>
    {:else}
      <p class="empty">The robot has not sent metadata yet.</p>
    {/each}
  </section>

  <section class="ui-panel">
    <header class="ui-panel-head"><h3 class="ui-panel-title">Classic dashboard</h3></header>
    <div class="ui-panel-body">
      <p class="text">The previous dashboard is still available while its layouts move into Dashboards.</p>
      <a class="ui-btn" href="?ui=classic">Open classic dashboard</a>
    </div>
  </section>
</div>

<style>
  .screen {
    display: grid;
    gap: var(--sp-3);
    max-width: 760px;
  }
  .empty,
  .text {
    margin: 0 0 var(--sp-3);
    padding: 0;
    color: var(--text-soft);
    font-size: var(--fs-sm);
  }
  .empty {
    padding: var(--sp-3);
    margin: 0;
  }
  a.ui-btn {
    text-decoration: none;
  }
</style>
