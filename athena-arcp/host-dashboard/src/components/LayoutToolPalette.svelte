<script lang="ts">
  import { LAYOUT_TOOL_KINDS, widgetKindLabel, type LayoutToolKind } from '../lib/dashboard';

  type Props = {
    query: string;
    onQueryInput: (value: string) => void;
    onAddLayoutTool: (kind: LayoutToolKind) => void;
    showHeader?: boolean;
  };

  const LAYOUT_DRAG_TYPE = 'application/x-arcp-layout-kind';
  const layoutToolDescriptions: Record<LayoutToolKind, string> = {
    layout_list: 'Vertical stack scaffold',
    layout_grid: 'Dense tiled scaffold',
    layout_accordion: 'Collapsible stack scaffold',
    layout_divider: 'Section separator',
    layout_spacer: 'Whitespace spacer',
    layout_section: 'Section title bar',
    layout_title: 'Large centered title text'
  };

  let { query, onQueryInput, onAddLayoutTool, showHeader = true }: Props = $props();

  let dragGhostEl = $state<HTMLDivElement | null>(null);

  const visibleTools = $derived.by(() => {
    const q = query.trim().toLowerCase();
    const all = LAYOUT_TOOL_KINDS.map((kind) => ({
      kind,
      label: widgetKindLabel(kind),
      description: layoutToolDescriptions[kind]
    }));
    if (!q) return all;

    return all.filter((tool) => {
      return (
        tool.label.toLowerCase().includes(q) ||
        tool.description.toLowerCase().includes(q) ||
        tool.kind.toLowerCase().includes(q)
      );
    });
  });

  function clearDragGhost() {
    if (!dragGhostEl) return;
    dragGhostEl.remove();
    dragGhostEl = null;
  }

  function makeDragGhost(label: string): HTMLDivElement {
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
    chip.textContent = 'layout';
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

  function startLayoutToolDrag(event: DragEvent, kind: LayoutToolKind) {
    if (!event.dataTransfer) return;
    event.dataTransfer.setData(LAYOUT_DRAG_TYPE, kind);
    event.dataTransfer.setData('text/plain', `layout:${kind}`);
    event.dataTransfer.effectAllowed = 'copy';

    const ghost = makeDragGhost(widgetKindLabel(kind));
    moveDragGhost(event);
    event.dataTransfer.setDragImage(ghost, 14, 14);
  }

  function endDrag() {
    clearDragGhost();
  }

  $effect(() => {
    return () => {
      clearDragGhost();
    };
  });
</script>

<aside class="layout-tool-palette panel">
  {#if showHeader}
    <header>
      <h2>Layout Tools</h2>
      <p>Drag or double-click to add scaffolding widgets.</p>
    </header>
  {/if}

  <div class="filters">
    <input
      class="search-input"
      value={query}
      placeholder="Search layout tools"
      oninput={(e) => onQueryInput((e.currentTarget as HTMLInputElement).value)}
    />
  </div>

  <div class="tools-list">
    {#if visibleTools.length === 0}
      <div class="empty">No layout tools match the search.</div>
    {:else}
      {#each visibleTools as tool (tool.kind)}
        <div
          class="tool-row"
          role="button"
          tabindex="0"
          draggable="true"
          title={tool.description}
          ondblclick={() => onAddLayoutTool(tool.kind)}
          ondragstart={(event) => startLayoutToolDrag(event, tool.kind)}
          ondrag={moveDragGhost}
          ondragend={endDrag}
          onkeydown={(event) => {
            if (event.key === 'Enter' || event.key === ' ') {
              event.preventDefault();
              onAddLayoutTool(tool.kind);
            }
          }}
        >
          <div class="tool-row-top">
            <strong>{tool.label}</strong>
            <span class="type-chip">layout</span>
          </div>
          <div class="tool-value">{tool.description}</div>
        </div>
      {/each}
    {/if}
  </div>
</aside>

<style>
  .layout-tool-palette {
    padding: 0.46rem;
    display: grid;
    grid-template-rows: auto auto 1fr;
    gap: 0.42rem;
    height: 100%;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 0.88rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.17rem 0 0;
    color: var(--text-soft);
    font-size: 0.73rem;
  }

  .filters {
    display: block;
  }

  .search-input {
    width: 100%;
    padding: 0.34rem 0.48rem;
    font-size: 0.74rem;
    border-radius: 7px;
  }

  .tools-list {
    display: grid;
    gap: 0.24rem;
    min-height: 0;
    overflow: auto;
    align-content: start;
  }

  .tool-row {
    border: 1px dashed rgba(99, 115, 140, 0.65);
    border-radius: 7px;
    padding: 0.26rem 0.28rem;
    display: grid;
    gap: 0.14rem;
    background: rgba(34, 41, 56, 0.55);
    cursor: pointer;
    user-select: none;
  }

  .tool-row:hover {
    border-color: rgba(180, 35, 45, 0.45);
    background: rgba(63, 34, 41, 0.45);
  }

  .tool-row-top {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.36rem;
  }

  .tool-row-top strong {
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

  .tool-value {
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
