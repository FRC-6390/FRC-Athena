<script lang="ts">
  import { widgetKindLabel } from '../../lib/dashboard';
  import { widgetKeyAction } from '../../lib/canvas/selection';
  import LayoutBlockBody from './LayoutBlockBody.svelte';
  import ResizeHandle from './ResizeHandle.svelte';
  import SignalBody from './SignalBody.svelte';
  import type { CanvasFrame, CanvasRuntime, FrameHandlers } from './types';

  let {
    frame,
    selected,
    dragging,
    editMode,
    runtime,
    handlers
  }: {
    frame: CanvasFrame;
    selected: boolean;
    dragging: boolean;
    editMode: boolean;
    runtime: CanvasRuntime;
    handlers: FrameHandlers;
  } = $props();

  const widget = $derived(frame.widget);

  function onKeydown(event: KeyboardEvent) {
    const action = widgetKeyAction(event.key);
    if (!action) return;
    event.preventDefault();
    handlers.key(action, widget);
  }
</script>

<div
  class={`widget-card ${frame.isLayout ? 'layout-card' : ''} ${frame.accordionOpen ? 'accordion-open' : ''} ${!frame.isLayout && frame.inLayout ? 'in-layout' : ''} ${selected ? 'active' : ''} ${dragging ? 'dragging-card' : ''}`}
  data-widget-id={widget.id}
  style={frame.style}
  role="button"
  tabindex="0"
  onclick={() => handlers.select(widget)}
  oncontextmenu={(event) => handlers.contextMenu(event, widget)}
  onkeydown={onKeydown}
>
  <button
    type="button"
    class="widget-head"
    onpointerdown={(event) => handlers.beginDrag(event, widget, 'move')}
    data-draggable={editMode}
    aria-label={`Move ${widget.title}`}
  >
    <div class="head-left">
      <strong>{widget.title}</strong>
      <span class="widget-type-chip">
        {widgetKindLabel(widget.kind)}
      </span>
    </div>
  </button>

  <div class="widget-body">
    {#if frame.isLayout}
      <LayoutBlockBody
        {widget}
        layout={frame.layout}
        gridCounts={frame.gridCounts}
        onToggleAccordion={handlers.toggleAccordion}
      />
    {:else}
      <SignalBody {widget} signal={frame.signal} unavailableHint={frame.unavailableHint} {runtime} />
    {/if}
  </div>

  {#if editMode}
    <ResizeHandle onpointerdown={(event) => handlers.beginDrag(event, widget, 'resize')} />
  {/if}
</div>

<style>
  .widget-card {
    position: absolute;
    left: 0;
    top: 0;
    box-sizing: border-box;
    border: 1px solid var(--border);
    border-radius: var(--radius-lg);
    background: var(--surface);
    padding: 0.12rem 0.28rem 0.24rem;
    display: grid;
    grid-template-rows: minmax(0.82rem, calc(var(--cell) * 0.46)) minmax(0, 1fr);
    gap: 0;
    cursor: pointer;
    overflow: hidden;
    box-shadow: 0 2px 8px rgba(0, 0, 0, 0.18);
    content-visibility: auto;
    contain-intrinsic-size: 220px 140px;
    contain: layout paint;
  }

  .widget-card.dragging-card {
    will-change: transform;
  }

  :global(.grid-surface.dragging) .widget-card {
    box-shadow: none;
  }

  .widget-card.layout-card {
    border-style: dashed;
    background: var(--surface-2);
    padding: 0.06rem 0.14rem 0.12rem;
    grid-template-rows: minmax(0.46rem, calc(var(--cell) * 0.2)) minmax(0, 1fr);
  }

  .widget-card.layout-card.accordion-open {
    overflow: visible;
  }

  .widget-card.in-layout {
    border-color: color-mix(in srgb, var(--info) 55%, transparent);
    background: var(--surface);
    box-shadow: inset 0 0 0 1px color-mix(in srgb, var(--info) 20%, transparent);
  }

  .widget-card.active {
    border-color: var(--brand);
    box-shadow: inset 0 0 0 1px color-mix(in srgb, var(--brand) 34%, transparent), 0 4px 12px rgba(0, 0, 0, 0.25);
  }

  .widget-head {
    display: flex;
    align-items: center;
    width: 100%;
    height: auto;
    min-height: 0;
    padding: 0 0 0.02rem;
    border: 0;
    border-radius: 0;
    border-bottom: 1px solid var(--border-subtle);
    background: transparent;
    color: inherit;
    text-align: left;
    font: inherit;
    line-height: 1.05;
  }

  .head-left {
    display: flex;
    align-items: center;
    justify-content: flex-start;
    gap: 0.28rem;
    min-width: 0;
    width: 100%;
  }

  .widget-head[data-draggable='true'] {
    cursor: grab;
    user-select: none;
    touch-action: none;
  }

  .widget-head strong {
    min-width: 0;
    flex: 1 1 auto;
    max-width: max(0px, calc(100% - 4.6rem));
    font-size: 0.72rem;
    color: var(--text-strong);
    text-overflow: ellipsis;
    white-space: nowrap;
    overflow: hidden;
    display: block;
  }

  .widget-type-chip {
    flex: 0 0 auto;
    margin-left: auto;
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.08rem 0.28rem;
    color: var(--text-soft);
    background: var(--surface-3);
    font-size: 0.62rem;
    line-height: 1;
    letter-spacing: 0.01em;
    text-transform: uppercase;
  }

  .widget-body {
    min-height: 0;
    display: flex;
    flex-direction: column;
    justify-content: safe center;
    gap: 0.28rem;
    overflow: auto;
  }

  .widget-card.layout-card .widget-body {
    justify-content: stretch;
  }

  .widget-card.layout-card .widget-head {
    border-bottom-width: 1px;
  }

  .widget-card.layout-card .head-left {
    gap: 0.14rem;
  }

  .widget-card.layout-card .widget-head strong {
    font-size: 0.58rem;
    max-width: max(0px, calc(100% - 2.8rem));
  }

  .widget-card.layout-card .widget-type-chip {
    padding: 0.04rem 0.16rem;
    font-size: 0.5rem;
  }

  .widget-card.layout-card.accordion-open .widget-body {
    overflow: visible;
  }
</style>
