<script lang="ts">
  import { GRID_GAP } from '../../lib/canvas/model';
  import CanvasWidgetFrame from './CanvasWidgetFrame.svelte';
  import type { CanvasFrame, CanvasRuntime, FrameHandlers } from './types';

  let {
    gridEl = $bindable(null),
    frames,
    runtime,
    handlers,
    selectedWidgetId,
    dragWidgetId,
    editMode,
    dragOver,
    cellSize,
    spacerWidth,
    spacerHeight,
    hasWidgets,
    onSurfaceDragOver,
    onSurfaceDragLeave,
    onSurfaceDrop,
    onToggleEditMode,
    onClear
  }: {
    gridEl?: HTMLDivElement | null;
    frames: CanvasFrame[];
    runtime: CanvasRuntime;
    handlers: FrameHandlers;
    selectedWidgetId: string | null;
    dragWidgetId: string | null;
    editMode: boolean;
    dragOver: boolean;
    cellSize: number;
    spacerWidth: number;
    spacerHeight: number;
    hasWidgets: boolean;
    onSurfaceDragOver: (event: DragEvent) => void;
    onSurfaceDragLeave: () => void;
    onSurfaceDrop: (event: DragEvent) => void;
    onToggleEditMode: () => void;
    onClear: () => void;
  } = $props();
</script>

<div
  class="grid-surface"
  class:drag-over={dragOver}
  class:dragging={dragWidgetId !== null}
  bind:this={gridEl}
  role="region"
  aria-label="Dashboard canvas grid"
  style={`--cell:${cellSize}px;--gap:${GRID_GAP}px;`}
  ondragover={onSurfaceDragOver}
  ondragleave={onSurfaceDragLeave}
  ondrop={onSurfaceDrop}
>
  <div class="grid-spacer" style={`width:${spacerWidth}px;height:${spacerHeight}px;`} aria-hidden="true"></div>

  {#if hasWidgets}
    <div class="canvas-actions">
      <button class="btn" data-active={editMode} onclick={onToggleEditMode}>
        {editMode ? 'Edit mode' : 'View mode'}
      </button>
      <button class="btn" disabled={!hasWidgets} onclick={onClear}>Clear</button>
    </div>
  {/if}

  {#each frames as frame (frame.widget.id)}
    <CanvasWidgetFrame
      {frame}
      selected={selectedWidgetId === frame.widget.id}
      dragging={dragWidgetId === frame.widget.id}
      {editMode}
      {runtime}
      {handlers}
    />
  {/each}
</div>

<style>
  .grid-surface {
    position: relative;
    width: 100%;
    border: 1px solid var(--border-subtle);
    border-radius: 10px;
    height: 100%;
    min-height: 0;
    background: var(--surface-2);
    overflow-y: auto;
    overflow-x: auto;
    scrollbar-gutter: stable both-edges;
    contain: layout paint;
  }

  .grid-surface.drag-over {
    border-color: rgba(180, 35, 45, 0.55);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.24);
  }

  .grid-spacer {
    min-width: 100%;
    min-height: 100%;
    background-image:
      linear-gradient(
        to right,
        rgba(153, 164, 180, 0.18) 1px,
        transparent 1px
      ),
      linear-gradient(
        to bottom,
        rgba(153, 164, 180, 0.14) 1px,
        transparent 1px
      );
    background-size: calc(var(--cell) + var(--gap)) calc(var(--cell) + var(--gap));
    background-position: 0 0;
    pointer-events: none;
  }

  .canvas-actions {
    position: absolute;
    top: 0.55rem;
    right: 0.55rem;
    z-index: 3;
    display: flex;
    gap: 0.34rem;
    background: rgba(20, 25, 36, 0.76);
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 0.3rem;
    backdrop-filter: blur(4px);
  }

  .canvas-actions .btn[data-active='true'] {
    border-color: rgba(180, 35, 45, 0.52);
    background: rgba(180, 35, 45, 0.24);
    color: #ffe7ea;
  }
</style>
