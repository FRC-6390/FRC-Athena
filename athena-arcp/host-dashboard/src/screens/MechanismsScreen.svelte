<script lang="ts">
  import type { Session } from '../lib/data/session';
  import { buildMechanismTree, flattenMechanisms, type MechanismNode } from '../lib/data/tree';
  import { summarize } from '../lib/mechanisms/model';
  import { reasonTone } from './mechanisms/tones';
  import MechanismDetail from './mechanisms/MechanismDetail.svelte';

  type Props = {
    session: Session;
    live: { readonly current: number };
    selected: string | null;
    onselect: (path: string) => void;
  };

  let { session, live, selected, onselect }: Props = $props();

  let filter = $state('');

  const nodes = $derived.by(() => {
    void live.current;
    const roots = buildMechanismTree(session.store.allSignals());
    const robot = roots.find((r) => r.name === 'robot');
    return flattenMechanisms(robot ? robot.children : roots);
  });
  const visible = $derived(nodes.filter((n) => n.path.toLowerCase().includes(filter.trim().toLowerCase())));
  const current = $derived(nodes.find((n) => n.path === selected) ?? nodes[0] ?? null);

  function reasonOf(node: MechanismNode): string {
    void live.current;
    const summary = summarize(node);
    if (!summary.reason) return '';
    return String(session.store.latestSample(summary.reason.signalId)?.value ?? '');
  }
</script>

<div class="screen">
  <aside class="ui-panel list">
    <div class="search">
      <input class="ui-input" placeholder="Filter mechanisms" bind:value={filter} aria-label="Filter mechanisms" />
    </div>
    <ul>
      {#each visible as node (node.path)}
        {@const reason = reasonOf(node)}
        <li>
          <button
            type="button"
            class:active={current?.path === node.path}
            style="padding-left:{12 + Math.max(0, node.depth - 1) * 14}px"
            onclick={() => onselect(node.path)}
          >
            <span class="ui-dot" data-tone={reason ? reasonTone(reason) : 'stale'}></span>
            <span class="text">
              <span class="name">{node.name}</span>
              {#if reason}<span class="reason">{reason}</span>{/if}
            </span>
          </button>
        </li>
      {:else}
        <li class="empty">No mechanisms{filter ? ' match' : ' published yet'}.</li>
      {/each}
    </ul>
  </aside>

  <div class="content">
    {#if current}
      <MechanismDetail node={current} {session} {live} />
    {:else}
      <div class="placeholder">Waiting for the robot's mechanism tree…</div>
    {/if}
  </div>
</div>

<style>
  .screen {
    display: grid;
    grid-template-columns: 260px minmax(0, 1fr);
    gap: var(--sp-3);
    height: 100%;
    min-height: 0;
  }
  .list {
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .search {
    padding: var(--sp-2);
    border-bottom: 1px solid var(--border);
  }
  .search input {
    width: 100%;
    font-family: var(--font-body);
  }
  ul {
    list-style: none;
    margin: 0;
    padding: var(--sp-1) 0;
    overflow: auto;
  }
  li button {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
    width: 100%;
    min-height: 40px;
    padding: 4px 12px;
    border: none;
    background: transparent;
    color: var(--text);
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  li button:hover {
    background: var(--surface-2);
  }
  li button.active {
    background: var(--surface-3);
    box-shadow: inset 2px 0 0 var(--brand);
  }
  .text {
    display: grid;
    min-width: 0;
  }
  .name {
    font-size: var(--fs-md);
    font-weight: 500;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .reason {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .empty {
    padding: var(--sp-3);
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
  .content {
    min-width: 0;
    min-height: 0;
    overflow: auto;
  }
  .placeholder {
    padding: var(--sp-5);
    color: var(--text-faint);
  }
</style>
