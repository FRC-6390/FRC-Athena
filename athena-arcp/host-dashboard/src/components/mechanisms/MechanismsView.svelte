<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import { buildMechanismTree, flattenMechanisms, type MechanismNode } from '../../lib/data/tree';
  import { summarize } from '../../lib/mechanisms/model';
  import { formatValue } from '../../lib/format';
  import MechanismPage from './MechanismPage.svelte';

  type Props = {
    session: Session;
    live: { readonly current: number };
  };

  let { session, live }: Props = $props();

  /** Every mechanism that publishes at least one signal of its own gets a generated page. */
  const pages = $derived.by(() => {
    void live.current;
    return flattenMechanisms(buildMechanismTree(session.store.allSignals())).filter(
      (node) => node.signals.length > 0
    );
  });

  let selectedPath = $state<string | null>(null);
  const selected = $derived<MechanismNode | null>(
    pages.find((p) => p.path === selectedPath) ?? pages[0] ?? null
  );

  function stateOf(node: MechanismNode): string {
    void live.current;
    const state = summarize(node).state;
    return state ? formatValue(session.store.latestSample(state.signalId)?.value) : '--';
  }
</script>

<div class="mechanisms">
  <nav class="list" aria-label="generated mechanism pages">
    {#if pages.length === 0}
      <p class="empty">Waiting for the robot dictionary.</p>
    {/if}
    {#each pages as node (node.path)}
      <button
        type="button"
        class="item"
        class:active={selected?.path === node.path}
        style="padding-left: {10 + node.depth * 12}px"
        onclick={() => (selectedPath = node.path)}
      >
        <span class="item-name">{node.name}</span>
        <span class="item-state">{stateOf(node)}</span>
      </button>
    {/each}
  </nav>

  <div class="content">
    {#if selected}
      {#key selected.path}
        <MechanismPage node={selected} {session} {live} />
      {/key}
    {/if}
  </div>
</div>

<style>
  .mechanisms {
    display: grid;
    grid-template-columns: minmax(170px, 220px) minmax(0, 1fr);
    gap: 14px;
    min-height: 0;
  }
  .list {
    display: grid;
    align-content: start;
    gap: 4px;
  }
  .item {
    display: flex;
    justify-content: space-between;
    gap: 8px;
    background: transparent;
    border: 1px solid transparent;
    border-radius: 8px;
    color: var(--text);
    padding: 7px 10px;
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  .item:hover {
    background: var(--surface-2);
  }
  .item.active {
    background: var(--surface-3);
    border-color: var(--border-emphasis);
    color: var(--text-strong);
  }
  .item-name {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .item-state {
    font-family: var(--font-mono);
    font-size: 11px;
    color: var(--text-soft);
  }
  .empty {
    color: var(--text-soft);
    font-size: 12px;
  }
  .content {
    min-width: 0;
  }
  @media (max-width: 720px) {
    .mechanisms {
      grid-template-columns: minmax(0, 1fr);
    }
  }
</style>
