<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faChevronRight } from '@fortawesome/free-solid-svg-icons';
  import type { SignalRow } from '../lib/arcp';

  type Props = {
    signals: SignalRow[];
    selectedId: number | null;
    query: string;
    onQueryInput: (value: string) => void;
    onSelectSignal: (signalId: number) => void;
    onAddSignal: (signal: SignalRow) => void;
    leafPath: (path: string) => string;
    showHeader?: boolean;
  };

  type ExplorerTab = 'arcp' | 'nt';

  type GroupBuildNode = {
    key: string;
    label: string;
    children: Map<string, GroupBuildNode>;
    signals: SignalRow[];
  };

  type GroupNode = {
    key: string;
    label: string;
    children: GroupNode[];
    signals: SignalRow[];
    signalCount: number;
  };

  type ExplorerGroupRow = {
    kind: 'group';
    key: string;
    label: string;
    depth: number;
    signalCount: number;
    open: boolean;
  };

  type ExplorerSignalRow = {
    kind: 'signal';
    signal: SignalRow;
    depth: number;
  };

  type ExplorerRow = ExplorerGroupRow | ExplorerSignalRow;

  let {
    signals,
    selectedId,
    query,
    onQueryInput,
    onSelectSignal,
    onAddSignal,
    leafPath,
    showHeader = true
  }: Props = $props();

  const SIGNAL_DRAG_TYPE = 'application/x-arcp-signal-id';
  const TOPIC_DRAG_TYPE = 'application/x-arcp-topic-path';

  const collator = new Intl.Collator(undefined, { sensitivity: 'base', numeric: true });

  let openGroups = $state<Set<string>>(new Set());
  let initializedArcpGroups = $state(false);
  let initializedNtGroups = $state(false);
  let explorerTab = $state<ExplorerTab>('arcp');
  let dragGhostEl: HTMLDivElement | null = null;

  function ntCompatPath(path: string): string {
    const trimmed = path.trim();
    if (!trimmed) return '';
    const normalized = trimmed.replace(/^\/+/, '');
    return normalized.startsWith('Athena/') || normalized === 'Athena'
      ? normalized
      : `Athena/${normalized}`;
  }

  function signalPath(signal: SignalRow): string {
    return explorerTab === 'nt' ? ntCompatPath(signal.path) : signal.path;
  }

  function splitPath(path: string): string[] {
    return path
      .split('/')
      .map((segment) => segment.trim())
      .filter((segment) => segment.length > 0);
  }

  function groupKeysForPath(path: string): string[] {
    const parts = splitPath(path);
    if (parts.length <= 1) return [];

    const keys: string[] = [];
    let prefix = '';
    for (const segment of parts.slice(0, -1)) {
      prefix = prefix ? `${prefix}/${segment}` : segment;
      keys.push(prefix);
    }
    return keys;
  }

  function makeBuildNode(key: string, label: string): GroupBuildNode {
    return {
      key,
      label,
      children: new Map<string, GroupBuildNode>(),
      signals: []
    };
  }

  function finalizeTree(node: GroupBuildNode): GroupNode {
    const children = [...node.children.values()]
      .map((child) => finalizeTree(child))
      .sort((a, b) => collator.compare(a.label, b.label));

    const sortedSignals = [...node.signals].sort((a, b) =>
      collator.compare(signalPath(a), signalPath(b))
    );
    const nestedSignalCount = children.reduce((count, child) => count + child.signalCount, 0);

    return {
      key: node.key,
      label: node.label,
      children,
      signals: sortedSignals,
      signalCount: sortedSignals.length + nestedSignalCount
    };
  }

  function buildTree(rows: SignalRow[], pathFor: (signal: SignalRow) => string): GroupNode {
    const root = makeBuildNode('', 'root');

    for (const signal of rows) {
      const parts = splitPath(pathFor(signal));
      const groups = parts.slice(0, -1);

      let cursor = root;
      let prefix = '';

      for (const group of groups) {
        prefix = prefix ? `${prefix}/${group}` : group;
        let child = cursor.children.get(group);
        if (!child) {
          child = makeBuildNode(prefix, group);
          cursor.children.set(group, child);
        }
        cursor = child;
      }

      cursor.signals.push(signal);
    }

    return finalizeTree(root);
  }

  const tree = $derived.by(() => buildTree(signals, signalPath));

  const allGroupKeys = $derived.by(() => {
    const keys: string[] = [];
    const walk = (node: GroupNode) => {
      for (const child of node.children) {
        keys.push(child.key);
        walk(child);
      }
    };
    walk(tree);
    return keys;
  });

  const treeRows = $derived.by(() => {
    const rows: ExplorerRow[] = [];

    const walk = (node: GroupNode, depth: number) => {
      for (const child of node.children) {
        const open = openGroups.has(child.key);
        rows.push({
          kind: 'group',
          key: child.key,
          label: child.label,
          depth,
          signalCount: child.signalCount,
          open
        });

        if (open) {
          walk(child, depth + 1);
        }
      }

      for (const signal of node.signals) {
        rows.push({
          kind: 'signal',
          signal,
          depth
        });
      }
    };

    walk(tree, 0);
    return rows;
  });

  function toggleGroup(key: string) {
    const next = new Set(openGroups);
    if (next.has(key)) {
      next.delete(key);
    } else {
      next.add(key);
    }
    openGroups = next;
  }

  function clearDragGhost() {
    if (!dragGhostEl) return;
    dragGhostEl.remove();
    dragGhostEl = null;
  }

  function normalizeTopicDragPath(groupKey: string): string {
    const trimmed = groupKey.trim();
    if (!trimmed) return '/';
    const withoutLeadingSlash = trimmed.replace(/^\/+/, '');
    const withoutAthenaRoot =
      explorerTab === 'nt'
        ? withoutLeadingSlash.replace(/^athena\/?/i, '')
        : withoutLeadingSlash;
    const withoutTrailingSlash = withoutAthenaRoot.replace(/\/+$/, '');
    return withoutTrailingSlash ? `/${withoutTrailingSlash}` : '/';
  }

  function makeDragGhost(label: string, chipText: string): HTMLDivElement {
    clearDragGhost();

    const ghost = document.createElement('div');
    ghost.style.position = 'fixed';
    ghost.style.left = '0px';
    ghost.style.top = '0px';
    ghost.style.display = 'inline-flex';
    ghost.style.alignItems = 'center';
    ghost.style.gap = '6px';
    ghost.style.padding = '6px 8px';
    ghost.style.borderRadius = '8px';
    ghost.style.border = '1px solid rgba(67, 80, 102, 0.95)';
    ghost.style.background = 'rgba(27, 33, 44, 0.96)';
    ghost.style.boxShadow = '0 8px 18px rgba(0, 0, 0, 0.34)';
    ghost.style.color = '#f5f7fb';
    ghost.style.fontFamily = "var(--font-body, 'IBM Plex Sans', sans-serif)";
    ghost.style.fontSize = '12px';
    ghost.style.pointerEvents = 'none';
    ghost.style.zIndex = '2147483647';

    const name = document.createElement('span');
    name.textContent = label;
    name.style.maxWidth = '200px';
    name.style.whiteSpace = 'nowrap';
    name.style.overflow = 'hidden';
    name.style.textOverflow = 'ellipsis';

    const chip = document.createElement('span');
    chip.textContent = chipText;
    chip.style.border = '1px solid rgba(67, 80, 102, 0.95)';
    chip.style.borderRadius = '999px';
    chip.style.background = 'rgba(43, 52, 69, 0.98)';
    chip.style.color = '#94a3b8';
    chip.style.fontSize = '10px';
    chip.style.letterSpacing = '0.02em';
    chip.style.padding = '2px 6px';
    chip.style.textTransform = 'uppercase';

    ghost.append(name, chip);
    document.body.appendChild(ghost);
    dragGhostEl = ghost;
    return ghost;
  }

  function moveDragGhost(event: DragEvent) {
    if (!dragGhostEl) return;
    if (event.clientX <= 0 && event.clientY <= 0) return;
    dragGhostEl.style.left = `${event.clientX + 14}px`;
    dragGhostEl.style.top = `${event.clientY + 14}px`;
  }

  function startSignalDrag(event: DragEvent, signal: SignalRow) {
    if (!event.dataTransfer) return;
    event.dataTransfer.setData(SIGNAL_DRAG_TYPE, String(signal.signal_id));
    event.dataTransfer.setData('text/plain', String(signal.signal_id));
    event.dataTransfer.effectAllowed = 'copy';

    const ghost = makeDragGhost(leafPath(signalPath(signal)), signal.signal_type);
    moveDragGhost(event);
    event.dataTransfer.setDragImage(ghost, 14, 14);
  }

  function startGroupDrag(event: DragEvent, group: ExplorerGroupRow) {
    if (!event.dataTransfer) return;
    const topicPath = normalizeTopicDragPath(group.key);
    event.dataTransfer.setData(TOPIC_DRAG_TYPE, topicPath);
    event.dataTransfer.setData('text/plain', `topic:${topicPath}`);
    event.dataTransfer.effectAllowed = 'copy';

    const signalCountLabel = group.signalCount === 1 ? '1 signal' : `${group.signalCount} signals`;
    const ghost = makeDragGhost(group.label, `topic · ${signalCountLabel}`);
    moveDragGhost(event);
    event.dataTransfer.setDragImage(ghost, 14, 14);
  }

  function endDrag() {
    clearDragGhost();
  }

  $effect(() => {
    if (explorerTab === 'arcp' && initializedArcpGroups) return;
    if (explorerTab === 'nt' && initializedNtGroups) return;
    if (tree.children.length === 0) return;

    openGroups = new Set(tree.children.map((group) => group.key));
    if (explorerTab === 'arcp') {
      initializedArcpGroups = true;
    } else {
      initializedNtGroups = true;
    }
  });

  $effect(() => {
    if (!query.trim()) return;

    const next = new Set(openGroups);
    let changed = false;
    for (const key of allGroupKeys) {
      if (next.has(key)) continue;
      next.add(key);
      changed = true;
    }

    if (changed) {
      openGroups = next;
    }
  });

  $effect(() => {
    if (selectedId === null) return;
    const selected = signals.find((signal) => signal.signal_id === selectedId);
    if (!selected) return;

    const ancestors = groupKeysForPath(signalPath(selected));
    if (ancestors.length === 0) return;

    const next = new Set(openGroups);
    let changed = false;
    for (const key of ancestors) {
      if (next.has(key)) continue;
      next.add(key);
      changed = true;
    }

    if (changed) {
      openGroups = next;
    }
  });

  $effect(() => {
    return () => {
      clearDragGhost();
    };
  });
</script>

<aside class="signal-browser panel">
  {#if showHeader}
    <header>
      <div>
        <h2>Data Explorer</h2>
        <p>{explorerTab === 'nt' ? 'Browse NetworkTables compatibility keys' : 'Browse ARCP signal catalog'}</p>
      </div>
      <span class="count-pill">{signals.length}</span>
    </header>
  {/if}

  <div class="explorer-tabs" role="tablist" aria-label="Explorer view">
    <button
      type="button"
      role="tab"
      class="explorer-tab"
      class:active={explorerTab === 'arcp'}
      aria-selected={explorerTab === 'arcp'}
      onclick={() => {
        explorerTab = 'arcp';
        initializedArcpGroups = false;
        openGroups = new Set();
      }}
    >
      ARCP
    </button>
    <button
      type="button"
      role="tab"
      class="explorer-tab"
      class:active={explorerTab === 'nt'}
      aria-selected={explorerTab === 'nt'}
      onclick={() => {
        explorerTab = 'nt';
        initializedNtGroups = false;
        openGroups = new Set();
      }}
    >
      NetworkTables
    </button>
  </div>

  <div class="filters">
    <input
      class="search-input"
      value={query}
      placeholder={explorerTab === 'nt' ? 'Search NT keys' : 'Search signals'}
      oninput={(e) => onQueryInput((e.currentTarget as HTMLInputElement).value)}
    />
  </div>

  <div class="signal-list">
    {#if signals.length === 0}
      <div class="empty">No entries match the current filters.</div>
    {:else}
      {#each treeRows as row (`${row.kind}-${row.kind === 'group' ? row.key : row.signal.signal_id}`)}
        {#if row.kind === 'group'}
          <button
            class="group-row"
            style={`--depth:${row.depth};`}
            draggable="true"
            title={`${normalizeTopicDragPath(row.key)} (${row.signalCount})`}
            aria-expanded={row.open}
            onclick={() => toggleGroup(row.key)}
            ondragstart={(event) => startGroupDrag(event, row)}
            ondrag={moveDragGhost}
            ondragend={endDrag}
          >
            <span class={`group-chevron ${row.open ? 'open' : ''}`} aria-hidden="true">
              <FontAwesomeIcon icon={faChevronRight} />
            </span>
            <span class="group-label">{row.label}</span>
            <span class="group-count">{row.signalCount}</span>
          </button>
        {:else}
          {@const signal = row.signal}
          <div
            class:selected={selectedId === signal.signal_id}
            class="signal-row"
            style={`--depth:${row.depth};`}
            role="button"
            tabindex="0"
            draggable="true"
            title={`${signalPath(signal)} (#${signal.signal_id})`}
            onclick={() => onSelectSignal(signal.signal_id)}
            ondblclick={() => onAddSignal(signal)}
            ondragstart={(event) => startSignalDrag(event, signal)}
            ondrag={moveDragGhost}
            ondragend={endDrag}
            onkeydown={(event) => {
              if (event.key === 'Enter' || event.key === ' ') {
                event.preventDefault();
                onSelectSignal(signal.signal_id);
              }
            }}
          >
            <div class="signal-row-top">
              <strong>{leafPath(signalPath(signal))}</strong>
              <span class="type-chip">{signal.signal_type}</span>
            </div>
            {#if explorerTab === 'nt'}
              <div class="signal-key">{signalPath(signal)}</div>
            {/if}
            <div class="signal-value">{signal.value}</div>
          </div>
        {/if}
      {/each}
    {/if}
  </div>
</aside>

<style>
  .signal-browser {
    padding: 0.46rem;
    display: grid;
    grid-template-rows: auto auto auto 1fr;
    gap: 0.42rem;
    height: 100%;
    min-height: 0;
  }

  header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 0.4rem;
  }

  header h2 {
    margin: 0;
    font-size: 0.88rem;
    letter-spacing: 0.01em;
    color: var(--text-strong);
  }

  header p {
    margin: 0.17rem 0 0;
    color: var(--text-soft);
    font-size: 0.73rem;
  }

  .count-pill {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.18rem 0.5rem;
    font-size: 0.72rem;
    color: var(--text-soft);
    background: var(--surface-3);
  }

  .filters {
    display: block;
  }

  .explorer-tabs {
    display: inline-flex;
    align-items: center;
    gap: 0.22rem;
    padding: 0.16rem;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    width: max-content;
  }

  .explorer-tab {
    border: 1px solid transparent;
    border-radius: 6px;
    background: transparent;
    color: var(--text-soft);
    font-size: 0.7rem;
    font-weight: 600;
    letter-spacing: 0.01em;
    padding: 0.22rem 0.5rem;
    line-height: 1.1;
  }

  .explorer-tab.active {
    border-color: rgba(180, 35, 45, 0.48);
    background: rgba(180, 35, 45, 0.2);
    color: #ffe7ea;
  }

  .search-input {
    width: 100%;
    padding: 0.34rem 0.48rem;
    font-size: 0.74rem;
    border-radius: 7px;
  }

  .signal-list {
    display: grid;
    gap: 0.24rem;
    min-height: 0;
    overflow: auto;
    padding-left: 0;
    padding-right: 0;
    direction: rtl;
    scrollbar-gutter: stable;
    align-content: start;
  }

  .signal-list > * {
    direction: ltr;
    width: 100%;
    justify-self: stretch;
  }

  .group-row {
    --depth: 0;
    --depth-indent: calc(var(--depth) * 0.52rem);
    width: 100%;
    max-width: calc(100% - var(--depth-indent));
    margin: 0;
    margin-left: var(--depth-indent);
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-2);
    color: var(--text);
    padding: 0.24rem 0.36rem;
    display: flex;
    align-items: center;
    gap: 0.28rem;
    text-align: left;
  }

  .group-row:hover {
    border-color: var(--border-emphasis);
  }

  .group-chevron {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 0.9rem;
    color: var(--text-soft);
    transform-origin: center center;
    transition: transform 120ms ease;
  }

  .group-chevron :global(svg) {
    width: 0.64rem;
    height: 0.64rem;
    display: block;
  }

  .group-chevron.open {
    transform: rotate(90deg);
  }

  .group-label {
    flex: 1 1 auto;
    min-width: 0;
    text-overflow: ellipsis;
    overflow: hidden;
    white-space: nowrap;
    font-size: 0.72rem;
    color: var(--text-strong);
  }

  .group-count {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.1rem 0.34rem;
    font-size: 0.64rem;
    color: var(--text-soft);
    background: var(--surface-3);
  }

  .signal-row {
    --depth: 0;
    --depth-indent: calc(var(--depth) * 0.52rem);
    max-width: calc(100% - var(--depth-indent));
    margin: 0;
    margin-left: var(--depth-indent);
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    padding: 0.26rem 0.36rem;
    display: grid;
    gap: 0.14rem;
    background: var(--surface);
    cursor: pointer;
    user-select: none;
  }

  .signal-row.selected {
    border-color: var(--brand);
    background: var(--brand-soft);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.2);
  }

  .signal-row-top {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.36rem;
  }

  .signal-row-top strong {
    flex: 1 1 auto;
    min-width: 0;
    font-size: 0.72rem;
    line-height: 1.1;
    color: var(--text-strong);
    text-overflow: ellipsis;
    overflow: hidden;
    white-space: nowrap;
  }

  .type-chip {
    flex: 0 0 auto;
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-3);
    color: var(--text-soft);
    font-size: 0.62rem;
    line-height: 1;
    padding: 0.1rem 0.32rem;
    text-transform: uppercase;
    letter-spacing: 0.02em;
  }

  .signal-value {
    font-family: var(--font-mono);
    font-size: 0.7rem;
    line-height: 1.2;
    white-space: normal;
    overflow-wrap: anywhere;
    color: var(--text-strong);
    background: var(--surface-3);
    border: 1px solid var(--border-subtle);
    border-radius: 5px;
    padding: 0.2rem 0.3rem;
  }

  .signal-key {
    font-family: var(--font-mono);
    font-size: 0.63rem;
    line-height: 1.2;
    white-space: nowrap;
    text-overflow: ellipsis;
    overflow: hidden;
    color: var(--text-soft);
    border: 1px dashed var(--border-subtle);
    border-radius: 5px;
    padding: 0.14rem 0.3rem;
    background: rgba(27, 33, 44, 0.66);
  }

  .empty {
    color: var(--text-soft);
    font-size: 0.8rem;
    padding: 0.8rem;
    text-align: center;
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    background: var(--surface-2);
  }
</style>
