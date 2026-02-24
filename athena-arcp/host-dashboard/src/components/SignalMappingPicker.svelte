<script lang="ts">
  import type { SignalRow } from '../lib/arcp';

  type PickerNodeBuild = {
    key: string;
    label: string;
    children: Map<string, PickerNodeBuild>;
    signals: SignalRow[];
  };

  type PickerNode = {
    key: string;
    label: string;
    children: PickerNode[];
    signals: SignalRow[];
  };

  type PickerGroupRow = {
    kind: 'group';
    key: string;
    label: string;
    depth: number;
    open: boolean;
  };

  type PickerSignalRow = {
    kind: 'signal';
    signal: SignalRow;
    depth: number;
  };

  type PickerRow = PickerGroupRow | PickerSignalRow;

  type Props = {
    open: boolean;
    title: string;
    signals: SignalRow[];
    selectedSignalId: number | null;
    allowNone?: boolean;
    noneLabel?: string;
    onPick: (signalId: number | null) => void;
    onClose: () => void;
  };

  let {
    open,
    title,
    signals,
    selectedSignalId,
    allowNone = true,
    noneLabel = 'None',
    onPick,
    onClose
  }: Props = $props();

  const collator = new Intl.Collator(undefined, { sensitivity: 'base', numeric: true });

  let query = $state('');
  let openGroups = $state<Set<string>>(new Set());

  function splitPath(path: string): string[] {
    return path
      .split('/')
      .map((segment) => segment.trim())
      .filter((segment) => segment.length > 0);
  }

  function leafPath(path: string): string {
    const parts = splitPath(path);
    return parts[parts.length - 1] ?? path;
  }

  function makeNode(key: string, label: string): PickerNodeBuild {
    return {
      key,
      label,
      children: new Map<string, PickerNodeBuild>(),
      signals: []
    };
  }

  function finalizeNode(node: PickerNodeBuild): PickerNode {
    const children = [...node.children.values()]
      .map((child) => finalizeNode(child))
      .sort((a, b) => collator.compare(a.label, b.label));
    const sortedSignals = [...node.signals].sort((a, b) => collator.compare(a.path, b.path));
    return {
      key: node.key,
      label: node.label,
      children,
      signals: sortedSignals
    };
  }

  function buildTree(rows: SignalRow[]): PickerNode {
    const root = makeNode('', 'root');

    for (const signal of rows) {
      const parts = splitPath(signal.path);
      const groups = parts.slice(0, -1);
      let cursor = root;
      let prefix = '';

      for (const group of groups) {
        prefix = prefix ? `${prefix}/${group}` : group;
        let child = cursor.children.get(group);
        if (!child) {
          child = makeNode(prefix, group);
          cursor.children.set(group, child);
        }
        cursor = child;
      }

      cursor.signals.push(signal);
    }

    return finalizeNode(root);
  }

  const filteredSignals = $derived.by(() => {
    const q = query.trim().toLowerCase();
    if (!q) return signals;
    return signals.filter((signal) => {
      return (
        signal.path.toLowerCase().includes(q) ||
        signal.signal_type.toLowerCase().includes(q) ||
        String(signal.signal_id).includes(q)
      );
    });
  });

  const tree = $derived.by(() => buildTree(filteredSignals));

  const treeRows = $derived.by(() => {
    const rows: PickerRow[] = [];
    const walk = (node: PickerNode, depth: number) => {
      for (const child of node.children) {
        const isOpen = openGroups.has(child.key);
        rows.push({
          kind: 'group',
          key: child.key,
          label: child.label,
          depth,
          open: isOpen
        });
        if (isOpen) {
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

  function closeIfBackdrop(event: PointerEvent) {
    if (event.currentTarget === event.target) {
      onClose();
    }
  }

  function toggleGroup(key: string) {
    const next = new Set(openGroups);
    if (next.has(key)) {
      next.delete(key);
    } else {
      next.add(key);
    }
    openGroups = next;
  }

  function pickSignal(signalId: number | null) {
    onPick(signalId);
    onClose();
  }

  $effect(() => {
    if (!open) return;
    query = '';
    openGroups = new Set(tree.children.map((entry) => entry.key));
  });

  $effect(() => {
    if (!open) return;
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Escape') return;
      event.preventDefault();
      onClose();
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  });
</script>

{#if open}
  <div class="picker-backdrop" role="presentation" onpointerdown={closeIfBackdrop}>
    <div class="picker-panel panel" role="dialog" aria-modal="true" aria-label={title}>
      <header>
        <h3>{title}</h3>
        <button class="btn" type="button" onclick={onClose}>Close</button>
      </header>

      <input
        class="picker-search"
        placeholder="Search supported signals"
        value={query}
        oninput={(event) => {
          query = (event.currentTarget as HTMLInputElement).value;
        }}
      />

      {#if allowNone}
        <button
          type="button"
          class="picker-none"
          class:active={selectedSignalId === null}
          onclick={() => pickSignal(null)}
        >
          {noneLabel}
        </button>
      {/if}

      <div class="picker-list">
        {#if treeRows.length === 0}
          <div class="picker-empty">No supported signals</div>
        {:else}
          {#each treeRows as row (`${row.kind}-${row.kind === 'group' ? row.key : row.signal.signal_id}`)}
            {#if row.kind === 'group'}
              <button
                type="button"
                class="group-row"
                style={`--depth:${row.depth};`}
                aria-expanded={row.open}
                onclick={() => toggleGroup(row.key)}
              >
                <span class={`caret ${row.open ? 'open' : ''}`} aria-hidden="true">▸</span>
                <span class="group-label">{row.label}</span>
              </button>
            {:else}
              {@const signal = row.signal}
              <button
                type="button"
                class="signal-row"
                class:active={signal.signal_id === selectedSignalId}
                style={`--depth:${row.depth};`}
                onclick={() => pickSignal(signal.signal_id)}
                title={`${signal.path} (#${signal.signal_id})`}
              >
                <div class="top">
                  <strong>{leafPath(signal.path)}</strong>
                  <span class="chip">{signal.signal_type}</span>
                </div>
                <div class="path">{signal.path}</div>
              </button>
            {/if}
          {/each}
        {/if}
      </div>
    </div>
  </div>
{/if}

<style>
  .picker-backdrop {
    position: fixed;
    inset: 0;
    z-index: 2500;
    background: rgba(7, 10, 16, 0.62);
    display: grid;
    place-items: center;
    padding: 1rem;
  }

  .picker-panel {
    width: min(760px, 96vw);
    max-height: min(86vh, 900px);
    min-height: min(420px, 70vh);
    padding: 0.56rem;
    display: grid;
    grid-template-rows: auto auto auto 1fr;
    gap: 0.4rem;
    overflow: hidden;
  }

  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.4rem;
  }

  header h3 {
    margin: 0;
    font-size: 0.82rem;
    color: var(--text-strong);
  }

  .picker-search {
    width: 100%;
    padding: 0.32rem 0.44rem;
    font-size: 0.72rem;
    border-radius: 7px;
  }

  .picker-none {
    justify-self: start;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-2);
    color: var(--text-soft);
    font-size: 0.7rem;
    padding: 0.24rem 0.48rem;
  }

  .picker-none.active {
    border-color: rgba(180, 35, 45, 0.52);
    background: rgba(180, 35, 45, 0.2);
    color: #ffe7ea;
  }

  .picker-list {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.24rem;
    align-content: start;
    padding-right: 0.14rem;
  }

  .group-row {
    --depth: 0;
    --depth-indent: calc(var(--depth) * 0.56rem);
    margin-left: var(--depth-indent);
    max-width: calc(100% - var(--depth-indent));
    width: 100%;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-2);
    color: var(--text);
    padding: 0.24rem 0.36rem;
    display: flex;
    align-items: center;
    gap: 0.24rem;
    text-align: left;
  }

  .group-row:hover {
    border-color: var(--border-emphasis);
  }

  .caret {
    width: 0.8rem;
    color: var(--text-soft);
    transition: transform 120ms ease;
  }

  .caret.open {
    transform: rotate(90deg);
  }

  .group-label {
    font-size: 0.72rem;
    color: var(--text-strong);
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .signal-row {
    --depth: 0;
    --depth-indent: calc(var(--depth) * 0.56rem);
    margin-left: var(--depth-indent);
    max-width: calc(100% - var(--depth-indent));
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface);
    color: var(--text);
    padding: 0.24rem 0.34rem;
    display: grid;
    gap: 0.13rem;
    text-align: left;
  }

  .signal-row:hover {
    border-color: var(--border-emphasis);
  }

  .signal-row.active {
    border-color: var(--brand);
    background: var(--brand-soft);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.2);
  }

  .top {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.3rem;
  }

  .top strong {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: 0.71rem;
  }

  .chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-3);
    color: var(--text-soft);
    font-size: 0.61rem;
    line-height: 1;
    padding: 0.1rem 0.3rem;
    text-transform: uppercase;
    letter-spacing: 0.02em;
    flex: 0 0 auto;
  }

  .path {
    font-family: var(--font-mono);
    font-size: 0.62rem;
    color: var(--text-soft);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .picker-empty {
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    background: var(--surface-2);
    color: var(--text-soft);
    font-size: 0.76rem;
    padding: 0.7rem;
    text-align: center;
  }
</style>
