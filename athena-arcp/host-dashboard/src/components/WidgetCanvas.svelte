<script lang="ts">
  import { onMount } from 'svelte';
  import type { SignalRow } from '../lib/arcp';
  import {
    clampLayout,
    isLayoutWidgetKind,
    type DashboardWidget,
    type LayoutToolKind,
    type WidgetConfigRecord,
    type WidgetLayout
  } from '../lib/dashboard';
  import { readLayoutAccordionConfig } from '../lib/widget-config';
  import { copyText } from '../lib/canvas/clipboard';
  import type { WidgetMenuAction, WidgetMenuState } from '../lib/canvas/context-menu';
  import { buildCanvasContext, type DragMode } from '../lib/canvas/model';
  import {
    canvasRowsFor,
    cellSizeFor,
    contentSizeFor,
    gridColumnsFor,
    layoutsEqual,
    pointToGridUnits,
    renderedExtent,
    scrollMetricsFor,
    visibleRowsFor
  } from '../lib/canvas/grid-math';
  import { gridSpecOf } from '../lib/canvas/containers';
  import { frameStyleFor } from '../lib/canvas/frame-style';
  import { renderOrder, toggledAccordionConfig } from '../lib/canvas/tree';
  import { advanceDrag, beginDragState, dragTargetFor, type DragState } from '../lib/canvas/drag';
  import { planDrop, planSurfaceDrop } from '../lib/canvas/drop';
  import { createSignalResolver } from '../lib/canvas/signal-resolve';
  import type { WidgetKeyAction } from '../lib/canvas/selection';
  import { acceptsDragTypes, parseDropPayload } from '../lib/canvas/drop-payload';
  import CanvasGrid from './canvas/CanvasGrid.svelte';
  import CanvasOverlays from './canvas/CanvasOverlays.svelte';
  import { measureLayoutBounds } from './canvas/measure';
  import { installCanvasListeners } from './canvas/listeners';
  import type { CanvasFrame, CanvasRuntime, FrameHandlers } from './canvas/types';

  type Props = {
    widgets: DashboardWidget[];
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    selectedWidgetId: string | null;
    widgetInputs: Record<string, string>;
    historyBySignal: Map<number, number[]>;
    historyTimeBySignal: Map<number, number[]>;
    editMode: boolean;
    onSelectWidget: (widgetId: string, signalId: number | null) => void;
    onToggleEditMode: () => void;
    onClearWidgets: () => void;
    onPatchWidgetLayout: (
      widgetId: string,
      layout: WidgetLayout,
      parentLayoutId?: string | null
    ) => void;
    onUpdateWidgetConfig: (widgetId: string, config: WidgetConfigRecord | undefined) => void;
    onRemoveWidget: (widgetId: string) => void;
    onWidgetInput: (widgetId: string, value: string) => void;
    onSendAction: (signalId: number) => void;
    onSendSet: (signalId: number, valueRaw: string) => void;
    onRequestInspector: () => void;
    onDropSignal: (
      signalId: number,
      position: Pick<WidgetLayout, 'x' | 'y'>,
      parentLayoutId?: string | null
    ) => void;
    onDropTopic: (
      topicPath: string,
      position: Pick<WidgetLayout, 'x' | 'y'>,
      parentLayoutId?: string | null
    ) => void;
    onDropLayoutTool: (
      kind: LayoutToolKind,
      position: Pick<WidgetLayout, 'x' | 'y'>,
      parentLayoutId?: string | null
    ) => void;
    onGridColumnsChange: (columns: number) => void;
  };

  let {
    widgets,
    signals,
    signalById,
    selectedWidgetId,
    widgetInputs,
    historyBySignal,
    historyTimeBySignal,
    editMode,
    onSelectWidget,
    onToggleEditMode,
    onClearWidgets,
    onPatchWidgetLayout,
    onUpdateWidgetConfig,
    onRemoveWidget,
    onWidgetInput,
    onSendAction,
    onSendSet,
    onRequestInspector,
    onDropSignal,
    onDropTopic,
    onDropLayoutTool,
    onGridColumnsChange
  }: Props = $props();

  let gridEl = $state<HTMLDivElement | null>(null);
  let gridWidth = $state(860);
  let gridHeight = $state(620);
  let drag = $state<DragState | null>(null);
  let contextMenu = $state<WidgetMenuState | null>(null);
  let dragOverSurface = $state(false);
  let copyToastVisible = $state(false);
  let copyToastTimer: ReturnType<typeof setTimeout> | null = null;
  let layoutMeasureTick = $state(0);
  let graphClockNowMs = $state(Date.now());

  const gridColumns = $derived(gridColumnsFor(widgets));
  const cellSize = $derived(cellSizeFor(gridWidth, gridColumns));
  const visibleRows = $derived(visibleRowsFor(gridHeight, cellSize));
  const contentSize = $derived(contentSizeFor(gridColumns, canvasRowsFor(widgets, visibleRows), cellSize));

  const measuredBounds = $derived.by(() => {
    layoutMeasureTick;
    gridEl;
    cellSize;
    gridColumns;
    return measureLayoutBounds(gridEl, widgets, cellSize, gridColumns);
  });

  const ctx = $derived(
    buildCanvasContext({
      widgets,
      gridColumns,
      drag,
      measuredBounds: (widget) => measuredBounds.get(widget.id) ?? clampLayout(widget.layout, gridColumns)
    })
  );

  const resolver = $derived(createSignalResolver(signals, signalById));

  const resolvedSignalByWidgetId = $derived.by(() => {
    const map = new Map<string, SignalRow | null>();
    for (const widget of widgets) {
      map.set(widget.id, isLayoutWidgetKind(widget.kind) ? null : resolver.resolveWidgetSignal(widget));
    }
    return map;
  });

  function signalIdOf(widget: DashboardWidget): number | null {
    return resolvedSignalByWidgetId.get(widget.id)?.signal_id ?? null;
  }

  /** Widgets to draw, in draw order, without the ones hidden by a collapsed accordion. */
  const frames = $derived.by((): CanvasFrame[] => {
    const result: CanvasFrame[] = [];
    for (const widget of renderOrder(widgets, ctx.byId)) {
      if (ctx.hidden(widget)) continue;
      const isLayout = isLayoutWidgetKind(widget.kind);
      const layout = ctx.layoutOf(widget);
      const spec = widget.kind === 'layout_grid' ? gridSpecOf(ctx, widget) : null;
      const accordionConfig = widget.kind === 'layout_accordion' ? readLayoutAccordionConfig(widget.config) : null;
      result.push({
        widget,
        layout,
        style: frameStyleFor({ ctx, drag, cellSize, widget, layout }),
        isLayout,
        signal: isLayout ? null : (resolvedSignalByWidgetId.get(widget.id) ?? null),
        unavailableHint: resolver.unavailableSignalHint(widget),
        accordionCollapsed: accordionConfig?.collapsed ?? false,
        gridCounts: spec ? { columns: spec.columns, rows: spec.rows } : null,
        inLayout: !isLayout && Boolean(widget.parentLayoutId),
        accordionOpen: accordionConfig !== null && !accordionConfig.collapsed
      });
    }
    return result;
  });

  const scroll = $derived(
    scrollMetricsFor(
      contentSize,
      renderedExtent(
        frames.map((frame) => ({ layout: frame.layout, withoutTrailingGap: Boolean(frame.widget.parentLayoutId) })),
        cellSize
      ),
      gridWidth,
      gridHeight
    )
  );

  const runtime: CanvasRuntime = $derived({
    signals,
    signalById,
    historyBySignal,
    historyTimeBySignal,
    nowMs: graphClockNowMs,
    widgetInputs,
    onWidgetInput,
    onSendSet,
    onSendAction
  });

  function pointerToCanvasUnits(clientX: number, clientY: number): { x: number; y: number } | null {
    if (!gridEl) return null;
    const rect = gridEl.getBoundingClientRect();
    return pointToGridUnits(
      clientX,
      clientY,
      { left: rect.left, top: rect.top, scrollLeft: gridEl.scrollLeft, scrollTop: gridEl.scrollTop },
      cellSize
    );
  }

  function closeContextMenu() {
    contextMenu = null;
  }

  function showCopyToast() {
    copyToastVisible = true;
    if (copyToastTimer) clearTimeout(copyToastTimer);
    copyToastTimer = setTimeout(() => {
      copyToastVisible = false;
      copyToastTimer = null;
    }, 1100);
  }

  async function copySignalValue(signalId: number) {
    const signal = signalById.get(signalId);
    if (!signal) return;
    if (await copyText(signal.value ?? '')) showCopyToast();
  }

  function beginDrag(event: PointerEvent, widget: DashboardWidget, mode: DragMode) {
    if (!editMode) return;
    if (event.button !== 0) return;
    closeContextMenu();

    const target = dragTargetFor(ctx, widget, mode);
    drag = beginDragState({
      target,
      mode,
      clientX: event.clientX,
      clientY: event.clientY,
      pointerOnCanvas: pointerToCanvasUnits(event.clientX, event.clientY),
      startLayout: ctx.layoutOf(target),
      frozenLayoutBounds: measureLayoutBounds(gridEl, widgets, cellSize, gridColumns)
    });
    event.preventDefault();
    event.stopPropagation();
  }

  function applyDragAt(pointerX: number, pointerY: number) {
    if (!drag) return;
    const next = advanceDrag(ctx, drag, pointerX, pointerY, pointerToCanvasUnits(pointerX, pointerY), cellSize);
    if (next === null) {
      drag = null;
    } else if (next !== drag) {
      drag = next;
    }
  }

  function endDrag() {
    if (!drag) return;
    const plan = planDrop(ctx, drag);
    if (plan?.patch) {
      onPatchWidgetLayout(plan.widgetId, plan.layout, plan.parentLayoutId);
    }
    drag = null;
  }

  function openContextMenu(event: MouseEvent, widget: DashboardWidget) {
    event.preventDefault();
    event.stopPropagation();
    onSelectWidget(widget.id, signalIdOf(widget));
    contextMenu = { widgetId: widget.id, signalId: signalIdOf(widget), x: event.clientX, y: event.clientY };
  }

  function selectWidget(widget: DashboardWidget) {
    closeContextMenu();
    onSelectWidget(widget.id, signalIdOf(widget));
  }

  function keyOnWidget(action: WidgetKeyAction, widget: DashboardWidget) {
    if (action === 'select') onSelectWidget(widget.id, signalIdOf(widget));
    else if (action === 'remove') onRemoveWidget(widget.id);
    else onRequestInspector();
  }

  function runMenuAction(action: WidgetMenuAction) {
    const menu = contextMenu;
    if (!menu) return;
    if (action === 'inspector') {
      onRequestInspector();
      closeContextMenu();
    } else if (action === 'copy') {
      if (menu.signalId !== null) {
        void copySignalValue(menu.signalId).finally(() => closeContextMenu());
      }
    } else {
      onRemoveWidget(menu.widgetId);
      closeContextMenu();
    }
  }

  const handlers: FrameHandlers = {
    select: selectWidget,
    contextMenu: openContextMenu,
    key: keyOnWidget,
    beginDrag,
    toggleAccordion: (widget) => onUpdateWidgetConfig(widget.id, toggledAccordionConfig(widget))
  };

  function onSurfaceDragOver(event: DragEvent) {
    const transfer = event.dataTransfer;
    if (!transfer || !acceptsDragTypes(transfer.types)) return;
    event.preventDefault();
    transfer.dropEffect = 'copy';
    dragOverSurface = true;
  }

  function onSurfaceDragLeave() {
    dragOverSurface = false;
  }

  function onSurfaceDrop(event: DragEvent) {
    const transfer = event.dataTransfer;
    dragOverSurface = false;
    if (!transfer) return;
    event.preventDefault();
    const payload = parseDropPayload((type) => transfer.getData(type));
    if (!payload) return;

    // Before the grid is measured, drops land at the origin of the canvas.
    let position: Pick<WidgetLayout, 'x' | 'y'> = { x: 1, y: 1 };
    let parentLayoutId: string | null = null;
    if (gridEl) {
      const rect = gridEl.getBoundingClientRect();
      const point = pointToGridUnits(
        event.clientX,
        event.clientY,
        { left: rect.left, top: rect.top, scrollLeft: gridEl.scrollLeft, scrollTop: gridEl.scrollTop },
        cellSize
      );
      const plan = planSurfaceDrop(ctx, point ?? { x: 1, y: 1 });
      position = plan.position;
      parentLayoutId = plan.parentLayoutId;
    }

    if (payload.kind === 'layout') onDropLayoutTool(payload.layoutKind, position, parentLayoutId);
    else if (payload.kind === 'topic') onDropTopic(payload.topicPath, position, parentLayoutId);
    else onDropSignal(payload.signalId, position, parentLayoutId);
  }

  onMount(() => {
    const observer = new ResizeObserver(() => {
      if (gridEl) {
        gridWidth = gridEl.clientWidth;
        gridHeight = gridEl.clientHeight;
      }
    });
    if (gridEl) {
      gridWidth = gridEl.clientWidth;
      gridHeight = gridEl.clientHeight;
      observer.observe(gridEl);
    }
    const graphClockTimer = window.setInterval(() => {
      graphClockNowMs = Date.now();
    }, 1000);

    const removeListeners = installCanvasListeners({
      isDragging: () => drag !== null,
      applyDragAt,
      endDrag,
      closeMenu: closeContextMenu,
      menu: () => contextMenu,
      copySignal: (signalId) => void copySignalValue(signalId).finally(() => closeContextMenu()),
      dragEnded: () => {
        dragOverSurface = false;
      }
    });

    return () => {
      observer.disconnect();
      window.clearInterval(graphClockTimer);
      removeListeners();
      if (copyToastTimer) {
        clearTimeout(copyToastTimer);
        copyToastTimer = null;
      }
    };
  });

  $effect(() => {
    onGridColumnsChange(gridColumns);
  });

  // Keep the drag's start and current layouts on the current column count.
  $effect(() => {
    if (!drag) return;
    const startLayout = clampLayout(drag.startLayout, gridColumns);
    const currentLayout = clampLayout(drag.currentLayout, gridColumns);
    if (layoutsEqual(startLayout, drag.startLayout) && layoutsEqual(currentLayout, drag.currentLayout)) return;
    drag = { ...drag, startLayout, currentLayout };
  });

  // Re-measure layout blocks after widgets change, once the DOM has been laid out.
  $effect(() => {
    widgets;
    if (typeof window === 'undefined' || typeof window.requestAnimationFrame !== 'function') return;
    const raf = window.requestAnimationFrame(() => {
      layoutMeasureTick += 1;
    });
    return () => window.cancelAnimationFrame(raf);
  });
</script>

<section class="dashboard-canvas panel">
  {#if widgets.length === 0}
    <div class="canvas-empty">
      <h3>Empty canvas</h3>
      <p>Use the floating Data Explorer to add your first panel.</p>
    </div>
  {/if}

  <CanvasGrid
    bind:gridEl
    {frames}
    {runtime}
    {handlers}
    selectedWidgetId={selectedWidgetId}
    dragWidgetId={drag?.widgetId ?? null}
    {editMode}
    dragOver={dragOverSurface}
    {cellSize}
    spacerWidth={scroll.scrollWidth}
    spacerHeight={scroll.scrollHeight}
    hasWidgets={widgets.length > 0}
    {onSurfaceDragOver}
    onSurfaceDragLeave={onSurfaceDragLeave}
    {onSurfaceDrop}
    {onToggleEditMode}
    onClear={onClearWidgets}
  />

  <CanvasOverlays menu={contextMenu} copyToastVisible={copyToastVisible} onAction={runMenuAction} />
</section>

<style>
  .dashboard-canvas {
    position: relative;
    padding: 0.46rem;
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: 1fr;
    overflow: hidden;
  }

  .canvas-empty {
    position: absolute;
    z-index: 2;
    top: 0.8rem;
    left: 0.8rem;
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    padding: 0.56rem 0.66rem;
    color: var(--text-soft);
    background: rgba(23, 29, 40, 0.86);
    backdrop-filter: blur(3px);
  }

  .canvas-empty h3 {
    margin: 0;
    color: var(--text-strong);
    font-size: 0.8rem;
  }

  .canvas-empty p {
    margin: 0.24rem 0 0;
    font-size: 0.72rem;
  }
</style>
