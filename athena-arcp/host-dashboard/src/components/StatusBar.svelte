<script lang="ts">
  type Props = {
    status: string;
    signals: number;
    updates: number;
    uptime: string;
    serverCpu: string;
    serverRss: string;
    hostCpu: string;
    hostRss: string;
  };

  let { status, signals, updates, uptime, serverCpu, serverRss, hostCpu, hostRss }: Props = $props();
</script>

<section class="status-bar panel">
  <span class="status">{status}</span>
  <span>signals {signals}</span>
  <span>updates total {updates}</span>
  <span>uptime {uptime}</span>
  <span>server cpu {serverCpu}</span>
  <span>server rss {serverRss}</span>
  <span>ui cpu {hostCpu}</span>
  <span>ui rss {hostRss}</span>
</section>

<style>
  .status-bar {
    display: flex;
    flex-wrap: wrap;
    gap: 0.45rem;
    align-items: center;
    padding: 0.45rem 0.58rem;
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  .status-bar span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-2);
    padding: 0.2rem 0.42rem;
  }

  .status-bar span.status {
    color: var(--text-strong);
    font-family: var(--font-display);
    background: var(--surface-3);
  }
</style>
