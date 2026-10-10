<script lang="ts">
  import type { Session } from '../lib/data/session';
  import type { SignalDescriptor } from '../lib/data/types';
  import ValueRow from '../ui/ValueRow.svelte';

  type Props = {
    session: Session;
    live: { readonly current: number };
  };

  let { session, live }: Props = $props();

  let filter = $state('');
  let writableOnly = $state(false);
  let collapsed = $state<Record<string, boolean>>({});

  /** Groups by mechanism path (or the first two segments for system signals) so the list reads like the robot tree. */
  const groups = $derived.by(() => {
    void live.current;
    const needle = filter.trim().toLowerCase();
    const map = new Map<string, SignalDescriptor[]>();
    for (const signal of session.store.allSignals()) {
      if (writableOnly && !signal.writable) continue;
      if (needle && !signal.path.toLowerCase().includes(needle)) continue;
      const key = signal.mechanismPath || signal.path.split('/').slice(0, 2).join('/');
      const list = map.get(key) ?? [];
      list.push(signal);
      map.set(key, list);
    }
    return [...map.entries()]
      .sort(([a], [b]) => a.localeCompare(b))
      .map(([key, signals]) => ({ key, signals: signals.sort((a, b) => a.path.localeCompare(b.path)) }));
  });
  const total = $derived(groups.reduce((n, g) => n + g.signals.length, 0));
  const relative = (group: string, signal: SignalDescriptor) =>
    signal.path.startsWith(`${group}/`) ? signal.path.slice(group.length + 1) : signal.path;
</script>

<section class="ui-panel screen">
  <div class="toolbar">
    <input class="ui-input search" placeholder="Filter by path" bind:value={filter} aria-label="Filter signals" />
    <label class="check"><input type="checkbox" bind:checked={writableOnly} /> Writable only</label>
    <span class="ui-spacer"></span>
    <span class="count ui-mono">{total} signals</span>
  </div>
  <div class="list">
    {#each groups as group (group.key)}
      <div class="group">
        <button type="button" class="group-head" onclick={() => (collapsed[group.key] = !collapsed[group.key])}>
          <span class="chev" class:open={!collapsed[group.key]}>›</span>
          <span class="gname ui-mono">{group.key}</span>
          <span class="gcount">{group.signals.length}</span>
        </button>
        {#if !collapsed[group.key]}
          {#each group.signals as signal (signal.signalId)}
            <ValueRow {signal} {session} {live} label={relative(group.key, signal)} />
          {/each}
        {/if}
      </div>
    {:else}
      <p class="empty">{filter ? 'No signals match.' : 'Waiting for the robot dictionary…'}</p>
    {/each}
  </div>
</section>

<style>
  .screen {
    display: flex;
    flex-direction: column;
    max-height: 100%;
    min-height: 0;
  }
  .toolbar {
    display: flex;
    align-items: center;
    gap: var(--sp-3);
    padding: var(--sp-2) var(--sp-3);
    border-bottom: 1px solid var(--border);
  }
  .search {
    width: min(420px, 100%);
    font-family: var(--font-body);
  }
  .check {
    display: inline-flex;
    gap: 6px;
    align-items: center;
    font-size: var(--fs-sm);
    color: var(--text-soft);
  }
  .count {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .list {
    overflow: auto;
    min-height: 0;
  }
  .group + .group {
    border-top: 1px solid var(--border-subtle);
  }
  .group-head {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
    width: 100%;
    height: 32px;
    padding: 0 var(--sp-3);
    border: none;
    background: var(--surface-2);
    color: var(--text);
    font: inherit;
    cursor: pointer;
    position: sticky;
    top: 0;
    z-index: 1;
  }
  .chev {
    display: inline-block;
    width: 10px;
    color: var(--text-faint);
    transition: transform 120ms;
  }
  .chev.open {
    transform: rotate(90deg);
  }
  .gname {
    font-size: var(--fs-sm);
    color: var(--text-strong);
  }
  .gcount {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .empty {
    margin: 0;
    padding: var(--sp-4);
    color: var(--text-faint);
  }
</style>
