<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import { buildMechanismTree, flattenMechanisms } from '../../lib/data/tree';
  import { deviceSections, deviceHealth, summarize } from '../../lib/mechanisms/model';
  import { formatValue } from '../../lib/format';

  type Props = {
    session: Session;
    live: { readonly current: number };
  };

  let { session, live }: Props = $props();

  /** One card per mechanism: state, reason, and any device warning, for a quick pit check. */
  const cards = $derived.by(() => {
    void live.current;
    const store = session.store;
    return flattenMechanisms(buildMechanismTree(store.allSignals()))
      .filter((node) => node.signals.length > 0)
      .map((node) => {
        const summary = summarize(node);
        const sample = (s: { signalId: number } | null) =>
          s ? store.latestSample(s.signalId)?.value : undefined;
        const warnings = deviceSections(node).flatMap((d) =>
          d.signals.filter((sig) => {
            const v = sample(sig);
            return deviceHealth(sig, typeof v === 'number' ? v : undefined) !== 'ok';
          })
        );
        return {
          path: node.path,
          name: node.name,
          state: formatValue(sample(summary.state)),
          reason: formatValue(sample(summary.reason)),
          warnings: warnings.map((w) => `${w.mechanismPath.split('/').pop()}/${w.name}`)
        };
      });
  });
</script>

<div class="pit">
  {#each cards as card (card.path)}
    <article class="card" class:warn={card.warnings.length > 0}>
      <header>
        <h3>{card.name}</h3>
        <span class="state">{card.state}</span>
      </header>
      <p class="reason">{card.reason}</p>
      {#if card.warnings.length > 0}
        <p class="warn-line">{card.warnings.join(', ')}</p>
      {/if}
    </article>
  {:else}
    <p class="empty">Waiting for the robot dictionary.</p>
  {/each}
</div>

<style>
  .pit {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(230px, 1fr));
    gap: 12px;
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px;
    display: grid;
    gap: 6px;
  }
  .card.warn {
    border-color: #b45309;
  }
  header {
    display: flex;
    justify-content: space-between;
    gap: 8px;
  }
  h3 {
    margin: 0;
    font-family: var(--font-display);
    font-size: 14px;
    color: var(--text-strong);
  }
  .state {
    font-family: var(--font-mono);
    font-size: 12px;
    color: var(--brand);
  }
  .reason {
    margin: 0;
    font-size: 12px;
    color: var(--text-soft);
  }
  .warn-line {
    margin: 0;
    font-size: 12px;
    color: #fbbf24;
    font-family: var(--font-mono);
  }
  .empty {
    color: var(--text-soft);
    font-size: 13px;
  }
</style>
