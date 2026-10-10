<script lang="ts">
  import type { Session } from '../lib/data/session';
  import type { SignalDescriptor } from '../lib/data/types';
  import { allWidgets } from '../lib/board/registry';
  import type { WidgetCategory } from '../lib/board/types';
  import { DRAG_MIME, type DragPayload } from './Board.svelte';
  import { buildSignalTree, signalsUnder, type TreeNode } from '../lib/board/signal-tree';

  type Props = {
    session: Session;
    live: { readonly current: number };
    /** Click-to-add fallback (keyboard users, touch screens). */
    onadd: (payload: DragPayload) => void;
  };

  let { session, live, onadd }: Props = $props();

  let tab = $state<'signals' | 'widgets'>('signals');
  let query = $state('');
  let open = $state<Record<string, boolean>>({});

  const tree = $derived.by(() => {
    void live.current;
    return buildSignalTree(session.store.allSignals(), query);
  });

  /** Folders start closed, except while searching or when there is only a little to show. */
  const isOpen = (node: TreeNode, depth: number) => open[node.key] ?? (query !== '' ? node.count <= 60 : depth === 0 && tree.length <= 2);

  function groupPayload(node: TreeNode): DragPayload {
    return { kind: 'group', unit: node.unit ?? 'folder', label: node.label, paths: signalsUnder(node).map((s) => s.path) };
  }

  const unitHint: Record<string, string> = {
    mechanism: 'Drag for a mechanism card',
    control: 'Drag for a control tuner',
    device: 'Drag for a device card',
    folder: 'Drag for a list of these signals'
  };

  const categories: WidgetCategory[] = ['Values', 'Controls', 'Charts', 'Robot', 'Match', 'Layout'];
  const widgets = $derived(
    categories
      .map((c) => ({
        category: c,
        defs: allWidgets()
          .filter((d) => d.category === c && (query === '' || d.name.toLowerCase().includes(query.toLowerCase())))
          .sort((a, b) => a.name.localeCompare(b.name))
      }))
      .filter((g) => g.defs.length > 0)
  );

  function drag(event: DragEvent, payload: DragPayload) {
    event.dataTransfer?.setData(DRAG_MIME, JSON.stringify(payload));
    if (event.dataTransfer) event.dataTransfer.effectAllowed = 'copy';
  }

  const typeLabel = (t: string) => (t.startsWith('struct:') ? t.slice(7) : t);
</script>

{#snippet branch(node: TreeNode, depth: number)}
  {#if node.signal}
    {@const signal = node.signal}
    <div
      class="item"
      role="button"
      tabindex="0"
      draggable="true"
      style="--depth:{depth}"
      title={signal.path}
      ondragstart={(e) => drag(e, { kind: 'signal', path: signal.path })}
      ondblclick={() => onadd({ kind: 'signal', path: signal.path })}
      onkeydown={(e) => e.key === 'Enter' && onadd({ kind: 'signal', path: signal.path })}
    >
      <span class="name">{node.label}</span>
      <span class="type" class:writable={signal.writable}>{typeLabel(signal.type)}</span>
    </div>
  {:else}
    {@const expanded = isOpen(node, depth)}
    <div
      class="ghead"
      class:unit={node.unit !== 'folder'}
      role="button"
      tabindex="0"
      draggable="true"
      style="--depth:{depth}"
      title={unitHint[node.unit ?? 'folder']}
      onclick={() => (open[node.key] = !expanded)}
      onkeydown={(e) => {
        if (e.key === 'Enter') onadd(groupPayload(node));
        if (e.key === ' ' || e.key === 'ArrowRight' || e.key === 'ArrowLeft') {
          e.preventDefault();
          open[node.key] = e.key === 'ArrowLeft' ? false : e.key === 'ArrowRight' ? true : !expanded;
        }
      }}
      ondblclick={() => onadd(groupPayload(node))}
      ondragstart={(e) => drag(e, groupPayload(node))}
    >
      <span class="chev" class:open={expanded}>›</span>
      <span class="gname">{node.label}</span>
      {#if node.unit === 'control' || node.unit === 'device'}<span class="tag">{node.unit}</span>{/if}
      <span class="count">{node.count}</span>
    </div>
    {#if expanded}
      {#each node.children as child (child.key)}
        {@render branch(child, depth + 1)}
      {/each}
    {/if}
  {/if}
{/snippet}

<aside class="drawer ui-panel">
  <div class="tabs">
    <button type="button" class:active={tab === 'signals'} onclick={() => (tab = 'signals')}>Signals</button>
    <button type="button" class:active={tab === 'widgets'} onclick={() => (tab = 'widgets')}>Widgets</button>
  </div>
  <div class="search">
    <input class="ui-input" placeholder={tab === 'signals' ? 'Search signals' : 'Search widgets'} bind:value={query} />
  </div>
  <p class="hint">
    {#if tab === 'signals'}Drag a signal, or a whole control, device or mechanism, onto the board. Double-click adds it.{:else}Drag onto the board, or double-click to add.{/if}
  </p>

  <div class="list">
    {#if tab === 'signals'}
      {#each tree as node (node.key)}
        {@render branch(node, 0)}
      {:else}
        <p class="empty">{query ? 'No signals match.' : 'Connect to a robot to see its signals.'}</p>
      {/each}
    {:else}
      {#each widgets as group (group.category)}
        <div class="cat">{group.category}</div>
        {#each group.defs as def (def.type)}
          <div
            class="item widget"
            role="button"
            tabindex="0"
            draggable="true"
            title={def.description}
            ondragstart={(e) => drag(e, { kind: 'widget', type: def.type })}
            ondblclick={() => onadd({ kind: 'widget', type: def.type })}
            onkeydown={(e) => e.key === 'Enter' && onadd({ kind: 'widget', type: def.type })}
          >
            <span class="name">{def.name}</span>
            <span class="desc">{def.description}</span>
          </div>
        {/each}
      {/each}
    {/if}
  </div>
</aside>

<style>
  .drawer {
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .tabs {
    display: grid;
    grid-template-columns: 1fr 1fr;
    border-bottom: 1px solid var(--border);
  }
  .tabs button {
    height: 36px;
    border: none;
    border-bottom: 2px solid transparent;
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-sm);
    font-weight: 600;
    cursor: pointer;
  }
  .tabs button.active {
    color: var(--text-strong);
    border-bottom-color: var(--brand);
  }
  .search {
    padding: var(--sp-2);
  }
  .search input {
    width: 100%;
    font-family: var(--font-body);
  }
  .hint {
    margin: 0;
    padding: 0 var(--sp-3) var(--sp-2);
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .list {
    overflow: auto;
    min-height: 0;
    padding-bottom: var(--sp-2);
  }
  .ghead {
    display: flex;
    align-items: center;
    gap: 6px;
    width: 100%;
    height: 28px;
    padding: 0 var(--sp-3) 0 calc(var(--sp-3) + var(--depth, 0) * 14px);
    box-sizing: border-box;
    border: none;
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: var(--fs-sm);
    font-weight: 600;
    text-align: left;
    cursor: pointer;
  }
  .ghead:hover {
    background: var(--surface-2);
  }
  .ghead.unit {
    cursor: grab;
  }
  .ghead:focus-visible {
    outline: 2px solid var(--info);
    outline-offset: -2px;
  }
  .tag {
    padding: 0 5px;
    border-radius: 3px;
    background: var(--info-soft);
    color: var(--info);
    font-size: 10px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }
  .chev {
    width: 10px;
    color: var(--text-faint);
    transition: transform 120ms;
  }
  .chev.open {
    transform: rotate(90deg);
  }
  .gname {
    flex: 1;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .count {
    font-size: var(--fs-xs);
    font-weight: 400;
    color: var(--text-faint);
  }
  .item {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 6px;
    margin: 1px var(--sp-2);
    padding: 4px 8px 4px calc(18px + var(--depth, 0) * 14px - var(--sp-2));
    border-radius: var(--radius-sm);
    font-size: var(--fs-sm);
    color: var(--text);
    cursor: grab;
    user-select: none;
  }
  .item:hover {
    background: var(--surface-3);
  }
  .item:focus-visible {
    outline: 2px solid var(--info);
  }
  .name {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .type {
    font-family: var(--font-mono);
    font-size: 10px;
    color: var(--text-faint);
  }
  .type.writable {
    color: var(--info);
  }
  .cat {
    padding: var(--sp-2) var(--sp-3) 4px;
    font-size: var(--fs-xs);
    font-weight: 600;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--text-faint);
  }
  .item.widget {
    grid-template-columns: 1fr;
    padding-left: 8px;
    gap: 1px;
  }
  .item.widget .name {
    font-weight: 600;
    color: var(--text-strong);
  }
  .desc {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .empty {
    margin: 0;
    padding: var(--sp-3);
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
