<script lang="ts" module>
  /** Drag payload type shared by the signal drawer, the widget palette and the board. */
  export const DRAG_MIME = 'application/x-athena-board';
  export type DragPayload = { kind: 'signal'; path: string } | { kind: 'widget'; type: string };
  export const CELL_HEIGHT = 56;
  export const CELL_MARGIN = 4;
</script>

<script lang="ts">
  import { onDestroy, onMount, tick } from 'svelte';
  import { GridStack, type GridStackNode } from 'gridstack';
  import 'gridstack/dist/gridstack.min.css';
  import type { Session } from '../lib/data/session';
  import type { BoardDoc, WidgetInstance } from '../lib/board/types';
  import WidgetFrame from './WidgetFrame.svelte';

  type Props = {
    board: BoardDoc;
    session: Session;
    live: { readonly current: number };
    editing: boolean;
    selectedId: string | null;
    onselect: (id: string | null) => void;
    /** Positions changed by dragging or resizing (one call per gesture). */
    onlayout: (changes: Array<{ id: string; x: number; y: number; w: number; h: number }>) => void;
    onremove: (id: string) => void;
    onduplicate: (id: string) => void;
    ondrop: (payload: DragPayload, cell: { x: number; y: number }) => void;
  };

  let { board, session, live, editing, selectedId, onselect, onlayout, onremove, onduplicate, ondrop }: Props = $props();

  let host = $state<HTMLDivElement | null>(null);
  let grid = $state<GridStack | null>(null);
  let dropHint = $state<{ x: number; y: number } | null>(null);

  onMount(() => {
    if (!host) return;
    grid = GridStack.init(
      {
        column: board.columns,
        cellHeight: CELL_HEIGHT,
        margin: CELL_MARGIN,
        float: true,
        animate: true,
        staticGrid: !editing,
        handle: '.drag-handle',
        minRow: 8,
        resizable: { handles: 'se,e,s,w,sw' }
      },
      host
    );
    grid.on('change', (_event: Event, nodes: GridStackNode[]) => {
      const changes = nodes
        .filter((n) => typeof n.id === 'string')
        .map((n) => ({ id: n.id as string, x: n.x ?? 0, y: n.y ?? 0, w: n.w ?? 1, h: n.h ?? 1 }));
      if (changes.length > 0) onlayout(changes);
    });
  });

  onDestroy(() => {
    grid?.destroy(false);
    grid = null;
  });

  $effect(() => {
    grid?.setStatic(!editing);
  });

  /** Hands each rendered item to gridstack; removal is reported back before Svelte drops the node. */
  function gridItem(node: HTMLElement, instance: WidgetInstance) {
    void tick().then(() => grid?.makeWidget(node, { id: instance.id, x: instance.x, y: instance.y, w: instance.w, h: instance.h }));
    return {
      update(next: WidgetInstance) {
        const n = (node as unknown as { gridstackNode?: GridStackNode }).gridstackNode;
        if (n && (n.x !== next.x || n.y !== next.y || n.w !== next.w || n.h !== next.h)) {
          grid?.update(node, { x: next.x, y: next.y, w: next.w, h: next.h });
        }
      },
      destroy() {
        grid?.removeWidget(node, false, false);
      }
    };
  }

  function cellAt(event: DragEvent): { x: number; y: number } {
    if (!host) return { x: 0, y: 0 };
    const rect = host.getBoundingClientRect();
    const colWidth = rect.width / board.columns;
    return {
      x: Math.max(0, Math.min(board.columns - 1, Math.floor((event.clientX - rect.left) / colWidth))),
      y: Math.max(0, Math.floor((event.clientY - rect.top) / CELL_HEIGHT))
    };
  }

  function isOurDrag(event: DragEvent): boolean {
    return event.dataTransfer?.types.includes(DRAG_MIME) ?? false;
  }

  function onDragOver(event: DragEvent) {
    if (!editing || !isOurDrag(event)) return;
    event.preventDefault();
    if (event.dataTransfer) event.dataTransfer.dropEffect = 'copy';
    dropHint = cellAt(event);
  }

  function onDrop(event: DragEvent) {
    dropHint = null;
    if (!editing || !isOurDrag(event)) return;
    event.preventDefault();
    try {
      const payload = JSON.parse(event.dataTransfer?.getData(DRAG_MIME) ?? '') as DragPayload;
      ondrop(payload, cellAt(event));
    } catch {
      // Not a board payload.
    }
  }
</script>

<div
  class="board"
  class:editing
  role="application"
  aria-label="Dashboard board"
  ondragover={onDragOver}
  ondragleave={() => (dropHint = null)}
  ondrop={onDrop}
  onpointerdown={(e) => {
    if (e.target === e.currentTarget || (e.target as HTMLElement).classList.contains('grid-stack')) onselect(null);
  }}
>
  {#if dropHint}
    <div
      class="drop-hint"
      style="left:{(dropHint.x / board.columns) * 100}%; top:{dropHint.y * CELL_HEIGHT}px; width:{(4 / board.columns) * 100}%; height:{2 * CELL_HEIGHT}px"
    ></div>
  {/if}
  <div class="grid-stack" bind:this={host}>
    {#if grid}
      {#each board.widgets as instance (instance.id)}
        <div class="grid-stack-item" use:gridItem={instance}>
          <div class="grid-stack-item-content">
            <WidgetFrame
              {instance}
              {session}
              {live}
              {editing}
              selected={selectedId === instance.id}
              onselect={() => onselect(instance.id)}
              onremove={() => onremove(instance.id)}
              onduplicate={() => onduplicate(instance.id)}
            />
          </div>
        </div>
      {/each}
    {/if}
  </div>
  {#if board.widgets.length === 0}
    <div class="empty">
      {#if editing}
        <p><b>Drag a signal or mechanism</b> from the left onto the board, or pick a widget from the palette.</p>
      {:else}
        <p>This board is empty. Press <b>Edit</b> to add widgets.</p>
      {/if}
    </div>
  {/if}
</div>

<style>
  .board {
    position: relative;
    min-height: 100%;
  }
  .board.editing {
    background-image: radial-gradient(circle, color-mix(in srgb, var(--border-emphasis) 55%, transparent) 1px, transparent 1.5px);
    background-size: calc(100% / 24) 56px;
    border-radius: var(--radius-lg);
  }
  .drop-hint {
    position: absolute;
    z-index: 5;
    border: 2px dashed var(--info);
    border-radius: var(--radius-lg);
    background: var(--info-soft);
    pointer-events: none;
  }
  .empty {
    position: absolute;
    inset: 80px 0 auto;
    display: grid;
    place-items: center;
    pointer-events: none;
    color: var(--text-soft);
    font-size: var(--fs-md);
  }
  .empty b {
    color: var(--text-strong);
  }
  :global(.board .grid-stack-item-content) {
    inset: 0 !important;
    overflow: visible !important;
  }
  :global(.board .grid-stack-placeholder > .placeholder-content) {
    border: 2px dashed var(--info) !important;
    border-radius: var(--radius-lg);
    background: var(--info-soft) !important;
  }
  :global(.board .ui-resizable-handle) {
    z-index: 4 !important;
  }
  :global(.board .ui-resizable-se) {
    width: 14px !important;
    height: 14px !important;
    right: 2px !important;
    bottom: 2px !important;
    background: none !important;
    border-right: 2px solid var(--border-emphasis);
    border-bottom: 2px solid var(--border-emphasis);
    border-radius: 0 0 4px 0;
    transform: none !important;
  }
</style>
