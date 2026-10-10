<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faMagnifyingGlass, faXmark } from '@fortawesome/free-solid-svg-icons';
  import { onMount, untrack } from 'svelte';
  import type { SignalRow } from '../../lib/arcp';
  import {
    DEFAULT_GRID_COLUMNS,
    applyFilters,
    clampLayout,
    createEmptyTab,
    defaultLeafWidgetKind,
    defaultWidgetKind,
    hardwareWidgetKindForPath,
    isLayoutWidgetKind,
    isWritableSignal,
    leafPath,
    makeLayoutWidget,
    makeWidget,
    parseLayoutImport,
    serializeLayoutExport,
    widgetKindLabel,
    type DashboardLayoutState,
    type DashboardWidget,
    type LayoutToolKind,
    type WidgetConfigRecord,
    type WidgetKind,
    type WidgetLayout
  } from '../../lib/dashboard';
  import {
    buildDefaultWidgetConfig,
    cloneWidgetConfig,
    readLayoutAccordionConfig,
    readLayoutGridConfig
  } from '../../lib/widget-config';
  import { classicKindsFor } from '../../lib/widgets/registry';
  import { downloadJsonFile, pickJsonFileText, shouldIgnoreGlobalHotkeys } from '../../lib/browser-io';
  import {
    advanceHistory,
    collectConfiguredSignalIds,
    EMPTY_HISTORY,
    type HistoryMaps
  } from '../../lib/canvas/history';
  import {
    isNtCompatPath,
    type CanvasCommands
  } from '../../lib/canvas-bridge';
  import WorkspaceTabs from '../../components/WorkspaceTabs.svelte';
  import SignalBrowser from '../../components/SignalBrowser.svelte';
  import LayoutToolPalette from '../../components/LayoutToolPalette.svelte';
  import WidgetCanvas from '../../components/WidgetCanvas.svelte';
  import InspectorPanel from '../../components/InspectorPanel.svelte';
  import FloatingDock from '../../components/FloatingDock.svelte';
  import DashboardContextMenu from '../../components/DashboardContextMenu.svelte';
  import type { CanvasDashboardsApi, SignalMapRequest } from './types';

  type CopiedWidgetSnapshot = {
    kind: WidgetKind;
    signalId: number;
    title: string;
    layout: WidgetLayout;
    parentLayoutId: string | undefined;
    config: WidgetConfigRecord | undefined;
    inputValue: string | null;
  };
  type DashboardContextMenuState = {
    x: number;
    y: number;
  };
  type CommitLayoutOptions = {
    recordHistory?: boolean;
    clearRedo?: boolean;
  };

  const HISTORY_LIMIT = 120;
  const ACCORDION_COLLAPSED_ROWS = 1;
  const DEFAULT_ACCORDION_EXPANDED_ROWS = 3;

  type Props = {
    /** The dashboard being edited. Bound: the canvas writes every change back through it. */
    layout: DashboardLayoutState;
    /** Live signals (publish plumbing already removed). */
    rows: SignalRow[];
    commands: CanvasCommands;
    /** Edit mode (drag, resize, configure). Bound. */
    editing?: boolean;
    /** When false the canvas keeps its state but renders nothing and ignores keys. */
    active?: boolean;
    gridColumns?: number;
    selectedId?: number | null;
    selectedWidgetId?: string | null;
    showInspector?: boolean;
    /** Called with every committed layout, so the host can persist it. */
    onLayoutChange?: (layout: DashboardLayoutState) => void;
  };

  let {
    layout = $bindable(),
    rows,
    commands,
    editing = $bindable(true),
    active = true,
    gridColumns = $bindable(DEFAULT_GRID_COLUMNS),
    selectedId = $bindable(null),
    selectedWidgetId = $bindable(null),
    showInspector = $bindable(false),
    onLayoutChange
  }: Props = $props();

  let lastError = $state('');
  let widgetInputs = $state<Record<string, string>>({});
  let undoStack = $state<DashboardLayoutState[]>([]);
  let redoStack = $state<DashboardLayoutState[]>([]);
  let copiedWidget = $state<CopiedWidgetSnapshot | null>(null);
  let dashboardMenu = $state<DashboardContextMenuState | null>(null);
  let pendingSignalMapRequest = $state<SignalMapRequest | null>(null);
  let showExplorer = $state(false);
  let showLayoutTools = $state(false);
  let query = $state('');
  let layoutToolQuery = $state('');
  let history = $state<HistoryMaps>(EMPTY_HISTORY);

  const signalById = $derived(new Map(rows.map((signal) => [signal.signal_id, signal])));
  const ntSignalRows = $derived(rows.filter((signal) => isNtCompatPath(signal.path)));
  const arcpSignalRows = $derived(rows.filter((signal) => !isNtCompatPath(signal.path)));

  const explorerSignals = $derived.by(() =>
    pendingSignalMapRequest
      ? applyFilters(pendingSignalMapRequest.candidates, query, 'all', 'all')
      : applyFilters(arcpSignalRows, query, 'all', 'all')
  );
  const filteredNtSignals = $derived.by(() =>
    pendingSignalMapRequest ? [] : applyFilters(ntSignalRows, query, 'all', 'all')
  );

  const activeTab = $derived(
    layout.tabs.find((tab) => tab.id === layout.activeTabId) ?? layout.tabs[0] ?? null
  );
  const widgets = $derived(activeTab?.widgets ?? []);
  const selectedSignal = $derived(selectedId === null ? null : signalById.get(selectedId) ?? null);
  const selectedWidget = $derived(
    selectedWidgetId ? widgets.find((widget) => widget.id === selectedWidgetId) ?? null : null
  );
  const tabSummaries = $derived(
    layout.tabs.map((tab) => ({ id: tab.id, name: tab.name, widgetCount: tab.widgets.length }))
  );

  const historySignalIds = $derived.by(() => {
    const ids = new Set<number>();
    for (const widget of widgets) {
      if (widget.kind === 'trend' && widget.signalId > 0) {
        ids.add(widget.signalId);
      }
      if (widget.kind === 'graph') {
        if (widget.signalId > 0) {
          ids.add(widget.signalId);
        }
        collectConfiguredSignalIds(widget.config, ids);
      }
    }
    return ids;
  });

  // Rolling history for trend and graph widgets, advanced whenever the live rows change.
  $effect(() => {
    const current = untrack(() => history);
    const next = advanceHistory(current, rows, historySignalIds, Date.now());
    if (next !== current) history = next;
  });

  function cloneLayoutState(state: DashboardLayoutState): DashboardLayoutState {
    return {
      activeTabId: state.activeTabId,
      tabs: state.tabs.map((tab) => ({
        id: tab.id,
        name: tab.name,
        widgets: tab.widgets.map((widget) => ({
          id: widget.id,
          signalId: widget.signalId,
          kind: widget.kind,
          title: widget.title,
          layout: { ...widget.layout },
          parentLayoutId: widget.parentLayoutId,
          config: cloneWidgetConfig(widget.config)
        }))
      }))
    };
  }

  function pushHistorySnapshot(stack: DashboardLayoutState[], snapshot: DashboardLayoutState): DashboardLayoutState[] {
    const next = [...stack, cloneLayoutState(snapshot)];
    if (next.length <= HISTORY_LIMIT) return next;
    return next.slice(next.length - HISTORY_LIMIT);
  }

  function layoutHasWidget(layout: DashboardLayoutState, widgetId: string): boolean {
    return layout.tabs.some((tab) => tab.widgets.some((widget) => widget.id === widgetId));
  }

  export function commitLayout(next: DashboardLayoutState, options: CommitLayoutOptions = {}) {
    const { recordHistory = true, clearRedo = true } = options;
    if (recordHistory) {
      undoStack = pushHistorySnapshot(undoStack, layout);
      if (clearRedo) {
        redoStack = [];
      }
    }

    layout = next;
    onLayoutChange?.(next);
  }

  function undoLayout() {
    if (undoStack.length === 0) return;

    const previous = undoStack[undoStack.length - 1];
    undoStack = undoStack.slice(0, undoStack.length - 1);
    redoStack = pushHistorySnapshot(redoStack, layout);
    layout = cloneLayoutState(previous);
    onLayoutChange?.(layout);

    if (selectedWidgetId && !layoutHasWidget(layout, selectedWidgetId)) {
      selectedWidgetId = null;
    }

  }

  function redoLayout() {
    if (redoStack.length === 0) return;

    const next = redoStack[redoStack.length - 1];
    redoStack = redoStack.slice(0, redoStack.length - 1);
    undoStack = pushHistorySnapshot(undoStack, layout);
    layout = cloneLayoutState(next);
    onLayoutChange?.(layout);

    if (selectedWidgetId && !layoutHasWidget(layout, selectedWidgetId)) {
      selectedWidgetId = null;
    }

  }

  function setActiveWidgets(nextWidgets: DashboardWidget[]) {
    if (!activeTab) return;
    const normalized = normalizeContainerChildren(nextWidgets);
    const nextTabs = layout.tabs.map((tab) =>
      tab.id === activeTab.id ? { ...tab, widgets: normalized } : tab
    );
    commitLayout({ ...layout, tabs: nextTabs });
  }

  type GridNormalizationSpec = {
    parent: WidgetLayout;
    columns: number;
    rows: number;
    xEdges: number[];
    yEdges: number[];
  };

  function buildAxisEdges(start: number, span: number, count: number): number[] {
    const safeCount = Math.max(1, Math.floor(count));
    const edges: number[] = [];
    for (let index = 0; index <= safeCount; index++) {
      edges.push(start + (index * span) / safeCount);
    }
    return edges;
  }

  function resolveGridNormalizationSpec(parent: DashboardWidget): GridNormalizationSpec | null {
    if (parent.kind !== 'layout_grid') return null;
    const parentLayout = clampLayout(parent.layout, gridColumns);
    const gridConfig = readLayoutGridConfig(parent.config, parentLayout.w, parentLayout.h);
    const columns = gridConfig.autoSize
      ? Math.max(1, Math.min(96, Math.round(parentLayout.w)))
      : Math.max(1, Math.floor(gridConfig.columns));
    const rows = gridConfig.autoSize
      ? Math.max(1, Math.min(96, Math.round(parentLayout.h)))
      : Math.max(1, Math.floor(gridConfig.rows));

    return {
      parent: parentLayout,
      columns,
      rows,
      xEdges: buildAxisEdges(parentLayout.x, parentLayout.w, columns),
      yEdges: buildAxisEdges(parentLayout.y, parentLayout.h, rows)
    };
  }

  function axisCellFromAbsoluteRange(
    rangeStart: number,
    rangeEnd: number,
    edges: number[]
  ): { index: number; span: number } {
    const count = Math.max(1, edges.length - 1);
    const start = Math.min(rangeStart, rangeEnd);
    const end = Math.max(rangeStart, rangeEnd);
    const epsilon = 1e-6;
    let first = -1;
    let last = -1;

    for (let idx = 0; idx < count; idx++) {
      const cellStart = edges[idx];
      const cellEnd = edges[idx + 1];
      if (end <= cellStart + epsilon || start >= cellEnd - epsilon) continue;
      if (first < 0) first = idx + 1;
      last = idx + 1;
    }

    if (first < 0) {
      const center = (start + end) / 2;
      let nearest = 1;
      let nearestDistance = Number.POSITIVE_INFINITY;
      for (let idx = 0; idx < count; idx++) {
        const cellStart = edges[idx];
        const cellEnd = edges[idx + 1];
        const cellCenter = (cellStart + cellEnd) / 2;
        const distance = Math.abs(center - cellCenter);
        if (distance < nearestDistance) {
          nearestDistance = distance;
          nearest = idx + 1;
        }
      }
      first = nearest;
      last = nearest;
    }

    return {
      index: first,
      span: Math.max(1, last - first + 1)
    };
  }

  function axisAbsoluteRangeFromCell(
    index: number,
    span: number,
    edges: number[]
  ): { start: number; span: number } {
    const count = Math.max(1, edges.length - 1);
    const clampedIndex = Math.max(1, Math.min(count, Math.floor(index)));
    const maxSpan = count - clampedIndex + 1;
    const clampedSpan = Math.max(1, Math.min(maxSpan, Math.floor(span)));
    const start = edges[clampedIndex - 1];
    const end = edges[clampedIndex + clampedSpan - 1];
    return {
      start,
      span: Math.max(1e-6, end - start)
    };
  }

  function snapLayoutToGridContainer(parent: DashboardWidget, layout: WidgetLayout): WidgetLayout {
    const spec = resolveGridNormalizationSpec(parent);
    if (!spec) return clampLayout(layout, gridColumns);

    const maxX = spec.parent.x + spec.parent.w;
    const maxY = spec.parent.y + spec.parent.h;
    const xStart = Math.max(spec.parent.x, Math.min(maxX, layout.x));
    const yStart = Math.max(spec.parent.y, Math.min(maxY, layout.y));
    const xEnd = Math.max(
      xStart,
      Math.min(maxX, layout.x + layout.w)
    );
    const yEnd = Math.max(
      yStart,
      Math.min(maxY, layout.y + layout.h)
    );

    const col = axisCellFromAbsoluteRange(xStart, xEnd, spec.xEdges);
    const row = axisCellFromAbsoluteRange(yStart, yEnd, spec.yEdges);

    const xRange = axisAbsoluteRangeFromCell(col.index, col.span, spec.xEdges);
    const yRange = axisAbsoluteRangeFromCell(row.index, row.span, spec.yEdges);

    return clampLayout(
      {
        x: xRange.start,
        y: yRange.start,
        w: xRange.span,
        h: yRange.span
      },
      gridColumns
    );
  }

  function normalizeContainerChildren(nextWidgets: DashboardWidget[]): DashboardWidget[] {
    if (nextWidgets.length === 0) return nextWidgets;

    const normalizedById = new Map<string, DashboardWidget>();
    for (const widget of nextWidgets) {
      normalizedById.set(widget.id, {
        ...widget,
        layout: clampLayout(widget.layout, gridColumns)
      });
    }

    const isContainerKind = (kind: WidgetKind): boolean =>
      kind === 'layout_list' || kind === 'layout_grid' || kind === 'layout_accordion';

    // Drop orphaned/invalid parent references before normalization.
    for (const [id, widget] of normalizedById) {
      const parentId = widget.parentLayoutId;
      if (!parentId) continue;
      const parent = normalizedById.get(parentId);
      if (!parent || !isContainerKind(parent.kind) || parentId === id) {
        normalizedById.set(id, {
          ...widget,
          parentLayoutId: undefined
        });
      }
    }

    // Break cycles to avoid unstable ordering/positioning loops.
    for (const [id, widget] of normalizedById) {
      if (!widget.parentLayoutId) continue;
      const seen = new Set<string>([id]);
      let cursor: string | undefined = widget.parentLayoutId;
      let cyclic = false;
      while (cursor) {
        if (seen.has(cursor)) {
          cyclic = true;
          break;
        }
        seen.add(cursor);
        const parent = normalizedById.get(cursor);
        cursor = parent?.parentLayoutId;
      }
      if (cyclic) {
        normalizedById.set(id, {
          ...widget,
          parentLayoutId: undefined
        });
      }
    }

    const depthCache = new Map<string, number>();
    const depthVisiting = new Set<string>();
    const hierarchyDepth = (widgetId: string): number => {
      const cached = depthCache.get(widgetId);
      if (cached !== undefined) return cached;
      if (depthVisiting.has(widgetId)) return 0;
      depthVisiting.add(widgetId);
      const widget = normalizedById.get(widgetId);
      const parentId = widget?.parentLayoutId;
      let depth = 0;
      if (parentId && normalizedById.has(parentId)) {
        depth = hierarchyDepth(parentId) + 1;
      }
      depthVisiting.delete(widgetId);
      depthCache.set(widgetId, depth);
      return depth;
    };

    const containerIds = nextWidgets
      .filter(
        (widget) =>
          widget.kind === 'layout_list' || widget.kind === 'layout_grid' || widget.kind === 'layout_accordion'
      )
      .map((widget) => widget.id)
      .sort((a, b) => {
        const depthDiff = hierarchyDepth(a) - hierarchyDepth(b);
        if (depthDiff !== 0) return depthDiff;
        const layoutA = normalizedById.get(a)?.layout ?? { x: 1, y: 1, w: 1, h: 1 };
        const layoutB = normalizedById.get(b)?.layout ?? { x: 1, y: 1, w: 1, h: 1 };
        return layoutA.y - layoutB.y || layoutA.x - layoutB.x || a.localeCompare(b);
      });

    for (const containerId of containerIds) {
      const parent = normalizedById.get(containerId);
      if (!parent || !isContainerKind(parent.kind)) continue;

      const children = Array.from(normalizedById.values())
        .filter((widget) => widget.parentLayoutId === parent.id)
        .sort(
          (a, b) =>
            a.layout.y - b.layout.y ||
            a.layout.x - b.layout.x ||
            a.id.localeCompare(b.id)
        );

      if (parent.kind === 'layout_accordion') {
        const [slotChild, ...overflow] = children;
        if (slotChild) {
          normalizedById.set(slotChild.id, {
            ...slotChild,
            parentLayoutId: parent.id
          });
        }
        for (const extra of overflow) {
          normalizedById.set(extra.id, {
            ...extra,
            parentLayoutId: undefined
          });
        }
        continue;
      }

      for (const child of children) {
        normalizedById.set(child.id, {
          ...child,
          parentLayoutId: parent.id
        });
      }
    }

    return nextWidgets.map((widget) => normalizedById.get(widget.id) ?? widget);
  }

  const TOPIC_HARDWARE_PRIORITY: WidgetKind[] = [
    'state_machine',
    'dio',
    'motor',
    'encoder',
    'imu'
  ];

  function normalizeTopicPath(path: string): string {
    const trimmed = path.trim();
    if (!trimmed) return '';
    const withoutLeadingSlash = trimmed.replace(/^\/+/, '').replace(/\\/g, '/');
    const withoutNtCompatRoot = withoutLeadingSlash
      .replace(/^athena\/nt4\/?/i, '')
      .replace(/^nt4\/?/i, '');
    const withoutAthenaRoot = withoutNtCompatRoot.replace(/^athena\/?/i, '');
    const withoutTrailingSlash = withoutAthenaRoot.replace(/\/+$/, '');
    return withoutTrailingSlash ? `/${withoutTrailingSlash}` : '';
  }

  function isSignalInTopic(signalPath: string, topicPath: string): boolean {
    if (!topicPath) return false;
    const normalizedSignalPath = normalizeTopicPath(signalPath);
    if (!normalizedSignalPath) return false;
    return (
      normalizedSignalPath === topicPath || normalizedSignalPath.startsWith(`${topicPath}/`)
    );
  }

  function resolveTopicDrop(
    topicPathRaw: string
  ): { signal: SignalRow; kind: WidgetKind; title: string } | null {
    const topicPath = normalizeTopicPath(topicPathRaw);
    if (!topicPath) return null;

    const scopedSignals = rows.filter((signal) => isSignalInTopic(signal.path, topicPath));
    if (scopedSignals.length === 0) return null;

    const topicLabel = leafPath(topicPath);
    const hardwareKindFromTopic = hardwareWidgetKindForPath(topicPath);
    if (hardwareKindFromTopic) {
      const anchorSignal =
        scopedSignals.find(
          (signal) => hardwareWidgetKindForPath(signal.path) === hardwareKindFromTopic
        ) ?? scopedSignals[0];
      return {
        signal: anchorSignal,
        kind: hardwareKindFromTopic,
        title: topicLabel
      };
    }

    for (const preferredKind of TOPIC_HARDWARE_PRIORITY) {
      const anchorSignal = scopedSignals.find(
        (signal) => hardwareWidgetKindForPath(signal.path) === preferredKind
      );
      if (!anchorSignal) continue;
      return {
        signal: anchorSignal,
        kind: preferredKind,
        title: topicLabel
      };
    }

    const fallbackSignal = scopedSignals.find((signal) => !isWritableSignal(signal)) ?? scopedSignals[0];
    return {
      signal: fallbackSignal,
      kind: defaultLeafWidgetKind(fallbackSignal),
      title: topicLabel
    };
  }

  function createWidget(
    signal: SignalRow,
    kind: WidgetKind,
    titleOverride = '',
    preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
    parentLayoutId?: string | null
  ) {
    const config = buildDefaultWidgetConfig(kind, signal, rows);
    let nextWidget = makeWidget(
      signal,
      kind,
      widgets,
      titleOverride,
      gridColumns,
      preferredPosition,
      parentLayoutId ?? undefined,
      config
    );

    const parentLayout =
      parentLayoutId === null || parentLayoutId === undefined
        ? null
        : widgets.find((entry) => entry.id === parentLayoutId) ?? null;
    const normalizedPreferredPosition = preferredPosition
      ? parentLayout?.kind === 'layout_grid'
        ? preferredPosition
        : {
            x: Math.max(1, Math.round(preferredPosition.x)),
            y: Math.max(1, Math.round(preferredPosition.y))
          }
      : undefined;

    // For direct drag-drop placement, honor the resolved drop position instead of global auto-placement.
    if (normalizedPreferredPosition) {
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: normalizedPreferredPosition.x,
            y: normalizedPreferredPosition.y
          },
          gridColumns
        )
      };
    }

    setActiveWidgets([nextWidget, ...widgets]);
    widgetInputs = {
      ...widgetInputs,
      [nextWidget.id]: signal.value === '-' ? '' : signal.value
    };
    selectedWidgetId = nextWidget.id;
    selectedId = signal.signal_id;
  }

  export function showAsWidgetKind(kind: WidgetKind) {
    if (!selectedSignal) return;

    if (!selectedWidget) {
      createWidget(selectedSignal, kind);
      return;
    }

    const anchorSignal = signalById.get(selectedWidget.signalId) ?? selectedSignal;
    const nextConfig = buildDefaultWidgetConfig(kind, anchorSignal, rows);

    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === selectedWidget.id
          ? {
              ...widget,
              kind,
              config: cloneWidgetConfig(nextConfig)
            }
          : widget
      )
    );
    selectedId = anchorSignal.signal_id;
  }

  function createLayoutWidget(
    kind: LayoutToolKind,
    preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
    parentLayoutId?: string | null
  ) {
    let nextWidget = makeLayoutWidget(
      kind,
      widgets,
      '',
      gridColumns,
      preferredPosition,
      undefined,
      parentLayoutId ?? undefined
    );

    const parentLayout =
      parentLayoutId === null || parentLayoutId === undefined
        ? null
        : widgets.find((entry) => entry.id === parentLayoutId) ?? null;
    const normalizedPreferredPosition = preferredPosition
      ? parentLayout?.kind === 'layout_grid'
        ? preferredPosition
        : {
            x: Math.max(1, Math.round(preferredPosition.x)),
            y: Math.max(1, Math.round(preferredPosition.y))
          }
      : undefined;

    if (normalizedPreferredPosition) {
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: normalizedPreferredPosition.x,
            y: normalizedPreferredPosition.y
          },
          gridColumns
        )
      };
    }

    setActiveWidgets([nextWidget, ...widgets]);
    selectedWidgetId = nextWidget.id;
    selectedId = null;
  }

  export function copySelectedWidget(): boolean {
    if (!selectedWidgetId) return false;

    const widget = widgets.find((entry) => entry.id === selectedWidgetId);
    if (!widget) return false;

    copiedWidget = {
      kind: widget.kind,
      signalId: widget.signalId,
      title: widget.title,
      layout: { ...widget.layout },
      parentLayoutId: widget.parentLayoutId,
      config: cloneWidgetConfig(widget.config),
      inputValue: widgetInputs[widget.id] ?? null
    };

    return true;
  }

  function closeDashboardMenu() {
    dashboardMenu = null;
  }

  function dashboardMenuStyle(menu: DashboardContextMenuState): string {
    const viewportWidth = typeof window === 'undefined' ? 1920 : window.innerWidth;
    const viewportHeight = typeof window === 'undefined' ? 1080 : window.innerHeight;
    const x = Math.min(menu.x, viewportWidth - 236);
    const y = Math.min(menu.y, viewportHeight - 276);
    return `left:${Math.max(8, x)}px;top:${Math.max(8, y)}px;`;
  }

  function openDashboardMenu(event: MouseEvent) {
    if (!active) return;

    const target = event.target as HTMLElement | null;
    if (!target) return;
    if (target.closest('.widget-card')) return;
    if (target.closest('.widget-context-menu')) return;
    if (target.closest('.floating-dock')) return;
    if (target.closest('.explorer-fab')) return;

    event.preventDefault();
    dashboardMenu = {
      x: event.clientX,
      y: event.clientY
    };
  }

  function openDataExplorerFromMenu() {
    pendingSignalMapRequest = null;
    showExplorer = true;
    query = '';
    closeDashboardMenu();
  }

  function closeExplorerDock() {
    showExplorer = false;
    pendingSignalMapRequest = null;
  }

  function requestSignalMapFromExplorer(request: SignalMapRequest) {
    pendingSignalMapRequest = request;
    showExplorer = true;
    query = '';
    if (selectedWidgetId !== null && selectedId === null) {
      const widget = widgets.find((entry) => entry.id === selectedWidgetId);
      if (widget && widget.signalId > 0) {
        selectedId = widget.signalId;
      }
    }
  }

  function openLayoutToolsFromMenu() {
    showLayoutTools = true;
    layoutToolQuery = '';
    closeDashboardMenu();
  }

  function addPageFromMenu() {
    addTab();
    closeDashboardMenu();
  }

  function renamePageFromMenu() {
    if (!activeTab) return;
    const nextName = window.prompt('Rename current page', activeTab.name);
    if (nextName === null) return;
    const trimmed = nextName.trim();
    if (!trimmed) return;
    renameTab(activeTab.id, trimmed);
    closeDashboardMenu();
  }

  function clearLayoutFromMenu() {
    clearWidgets();
    closeDashboardMenu();
  }

  function suggestedLayoutFilename(): string {
    const now = new Date();
    const stamp = now
      .toISOString()
      .replace(/[:]/g, '-')
      .replace(/\..+$/, '');
    return `arcp-ui-${stamp}.json`;
  }


  export async function exportLayoutToFile() {
    try {
      const serialized = serializeLayoutExport(layout);
      downloadJsonFile(suggestedLayoutFilename(), serialized);
      lastError = '';
    } catch (err) {
      lastError = `save failed: ${String(err)}`;
    }
  }

  export async function importLayoutFromFile() {
    try {
      const raw = await pickJsonFileText();
      if (!raw) return;

      const hasExistingWidgets = layout.tabs.some((tab) => tab.widgets.length > 0);
      if (hasExistingWidgets) {
        const confirmed = window.confirm(
          'Load UI JSON and replace the current layout? This action can be undone with Ctrl+Z.'
        );
        if (!confirmed) return;
      }

      const imported = parseLayoutImport(raw);
      commitLayout(imported, { recordHistory: true, clearRedo: true });
      selectedWidgetId = null;
      selectedId = null;
      widgetInputs = {};
      lastError = '';
    } catch (err) {
      lastError = `load failed: ${String(err)}`;
    }
  }

  function importLayoutFromMenu() {
    closeDashboardMenu();
    void importLayoutFromFile();
  }

  function exportLayoutFromMenu() {
    closeDashboardMenu();
    void exportLayoutToFile();
  }


  export function pasteCopiedWidget(): boolean {
    if (!copiedWidget) return false;

    const preferred = {
      x: copiedWidget.layout.x + 1,
      y: copiedWidget.layout.y + 1
    };

    if (isLayoutWidgetKind(copiedWidget.kind)) {
      const nextWidget = makeLayoutWidget(
        copiedWidget.kind,
        widgets,
        copiedWidget.title,
        gridColumns,
        preferred,
        cloneWidgetConfig(copiedWidget.config)
      );
      setActiveWidgets([nextWidget, ...widgets]);
      selectedWidgetId = nextWidget.id;
      selectedId = null;
      return true;
    }

    const signal = signalById.get(copiedWidget.signalId);
    if (!signal) {
      lastError = 'cannot paste: source signal is unavailable';
      return false;
    }

    const nextWidget = makeWidget(
      signal,
      copiedWidget.kind,
      widgets,
      copiedWidget.title,
      gridColumns,
      preferred,
      copiedWidget.parentLayoutId,
      cloneWidgetConfig(copiedWidget.config)
    );

    setActiveWidgets([nextWidget, ...widgets]);
    selectedWidgetId = nextWidget.id;
    selectedId = signal.signal_id;
    widgetInputs = {
      ...widgetInputs,
      [nextWidget.id]: copiedWidget.inputValue ?? signal.value
    };
    return true;
  }

  export function removeWidget(widgetId: string) {
    const removingLayout = widgets.find((widget) => widget.id === widgetId);
    const nextInputs = { ...widgetInputs };
    delete nextInputs[widgetId];
    widgetInputs = nextInputs;

    const filtered = widgets
      .filter((widget) => widget.id !== widgetId)
      .map((widget) => {
        if (!removingLayout || !isLayoutWidgetKind(removingLayout.kind)) {
          return widget;
        }
        if (widget.parentLayoutId !== widgetId) return widget;
        return {
          ...widget,
          parentLayoutId: undefined
        };
      });

    setActiveWidgets(filtered);
    if (selectedWidgetId === widgetId) {
      selectedWidgetId = null;
    }
  }

  function patchWidgetLayout(
    widgetId: string,
    layout: WidgetLayout,
    parentLayoutId?: string | null
  ) {
    const moving = widgets.find((widget) => widget.id === widgetId);
    if (!moving) return;

    const clamped = clampLayout(layout, gridColumns);
    const dx = clamped.x - moving.layout.x;
    const dy = clamped.y - moving.layout.y;
    const hasDelta = dx !== 0 || dy !== 0;
    const widgetsById = new Map(widgets.map((widget) => [widget.id, widget]));

    const isDescendantOfMoving = (candidate: DashboardWidget): boolean => {
      let cursor = candidate.parentLayoutId;
      while (cursor) {
        if (cursor === widgetId) return true;
        const parent = widgetsById.get(cursor);
        cursor = parent?.parentLayoutId;
      }
      return false;
    };

    setActiveWidgets(
      widgets.map((widget) => {
        if (widget.id === widgetId) {
          return {
            ...widget,
            layout: clamped,
            parentLayoutId: parentLayoutId === undefined ? widget.parentLayoutId : parentLayoutId ?? undefined
          };
        }

        if (isLayoutWidgetKind(moving.kind) && hasDelta && isDescendantOfMoving(widget)) {
          return {
            ...widget,
            layout: clampLayout(
              {
                ...widget.layout,
                x: widget.layout.x + dx,
                y: widget.layout.y + dy
              },
              gridColumns
            )
          };
        }

        return widget;
      })
    );
  }

  export function updateWidgetConfig(widgetId: string, config: WidgetConfigRecord | undefined) {
    const target = widgets.find((widget) => widget.id === widgetId);
    if (!target) return;

    const nextConfig = cloneWidgetConfig(config);
    if (target.kind !== 'layout_accordion') {
      setActiveWidgets(
        widgets.map((widget) =>
          widget.id === widgetId
            ? {
                ...widget,
                config: nextConfig
              }
            : widget
        )
      );
      return;
    }

    const nextConfigRecord =
      nextConfig && typeof nextConfig === 'object' && !Array.isArray(nextConfig)
        ? nextConfig
        : {};
    const previousAccordion = readLayoutAccordionConfig(target.config);
    const nextAccordion = readLayoutAccordionConfig(nextConfigRecord);
    const previousCollapsed = previousAccordion.collapsed;
    const nextCollapsed = nextAccordion.collapsed;
    const currentRows = Math.max(1, Math.round(target.layout.h));

    let expandedRows = nextAccordion.expandedRows ?? previousAccordion.expandedRows;
    if (expandedRows === null) {
      expandedRows = previousCollapsed
        ? Math.max(DEFAULT_ACCORDION_EXPANDED_ROWS, currentRows)
        : currentRows;
    }
    expandedRows = Math.max(1, Math.min(96, Math.round(expandedRows)));

    let nextLayout = clampLayout(target.layout, gridColumns);
    if (!previousCollapsed && nextCollapsed) {
      expandedRows = Math.max(1, currentRows);
      nextLayout = clampLayout(
        {
          ...nextLayout,
          h: ACCORDION_COLLAPSED_ROWS
        },
        gridColumns
      );
    } else if (previousCollapsed && !nextCollapsed) {
      expandedRows = Math.max(
        ACCORDION_COLLAPSED_ROWS,
        Math.min(96, Math.round(nextAccordion.expandedRows ?? previousAccordion.expandedRows ?? DEFAULT_ACCORDION_EXPANDED_ROWS))
      );
      nextLayout = clampLayout(
        {
          ...nextLayout,
          h: expandedRows
        },
        gridColumns
      );
    } else if (!nextCollapsed) {
      expandedRows = Math.max(ACCORDION_COLLAPSED_ROWS, currentRows);
    }

    const patchedConfig: WidgetConfigRecord = {
      ...nextConfigRecord,
      collapsed: nextCollapsed,
      expandedRows
    };

    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === widgetId
          ? {
              ...widget,
              layout: nextLayout,
              config: patchedConfig
            }
          : widget
      )
    );
  }

  export function updateWidgetTitle(widgetId: string, title: string) {
    const nextTitle = title.trim();
    if (!nextTitle) return;
    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === widgetId
          ? {
              ...widget,
              title: nextTitle
            }
          : widget
      )
    );
  }

  function clearWidgets() {
    if (widgets.length === 0) return;
    const confirmed = window.confirm(
      `Clear all widgets from this dashboard tab? (${widgets.length} item${widgets.length === 1 ? '' : 's'})`
    );
    if (!confirmed) return;

    selectedWidgetId = null;
    setActiveWidgets([]);
  }

  function setWidgetInput(widgetId: string, value: string) {
    widgetInputs = { ...widgetInputs, [widgetId]: value };
  }

  function handleGridColumnsChange(columns: number) {
    if (!Number.isFinite(columns) || columns < 1 || columns === gridColumns) {
      return;
    }
    gridColumns = columns;

    if (!activeTab || activeTab.widgets.length === 0) {
      return;
    }

    const nextWidgets = activeTab.widgets.map((widget) => ({
      ...widget,
      layout: clampLayout(widget.layout, columns)
    }));

    const changed = nextWidgets.some((widget, index) => {
      const prev = activeTab.widgets[index];
      return (
        widget.layout.x !== prev.layout.x ||
        widget.layout.y !== prev.layout.y ||
        widget.layout.w !== prev.layout.w ||
        widget.layout.h !== prev.layout.h
      );
    });

    if (changed) {
      setActiveWidgets(nextWidgets);
    }
  }

  function addTab() {
    const tab = createEmptyTab(`Dashboard ${layout.tabs.length + 1}`);
    commitLayout({
      activeTabId: tab.id,
      tabs: [...layout.tabs, tab]
    });
    selectedWidgetId = null;
    selectedId = null;
  }

  function removeTab(tabId: string) {
    if (layout.tabs.length <= 1) return;

    const nextTabs = layout.tabs.filter((tab) => tab.id !== tabId);
    const nextActive =
      layout.activeTabId === tabId ? nextTabs[0].id : layout.activeTabId;

    commitLayout({
      activeTabId: nextActive,
      tabs: nextTabs
    });

    selectedWidgetId = null;
  }

  function renameTab(tabId: string, name: string) {
    const nextTabs = layout.tabs.map((tab) => (tab.id === tabId ? { ...tab, name } : tab));
    commitLayout({ ...layout, tabs: nextTabs });
  }

  export function openInspectorForSelection() {
    if (selectedWidgetId !== null && selectedId === null) {
      const widget = widgets.find((entry) => entry.id === selectedWidgetId);
      if (widget && widget.signalId > 0) {
        selectedId = widget.signalId;
      }
    }
    showInspector = true;
  }

  async function sendAction(signalId: number) {
    lastError = (await commands.triggerAction(signalId)) ?? '';
  }

  async function sendSet(signalId: number, valueRaw: string) {
    lastError = (await commands.setValue(signalId, valueRaw)) ?? '';
  }

  // Keyboard shortcuts for the canvas: edit history, copy/paste, delete, inspector, file export/import.
  function onKeyDown(event: KeyboardEvent) {
    if (!active) return;
    const key = event.key.toLowerCase();

    if (key === 'escape' && dashboardMenu) {
      event.preventDefault();
      closeDashboardMenu();
      return;
    }

    if (event.defaultPrevented || shouldIgnoreGlobalHotkeys(event.target)) {
      return;
    }

    const hasAccel = event.ctrlKey || event.metaKey;

    if (hasAccel && key === 's' && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      void exportLayoutToFile();
      return;
    }

    if (hasAccel && key === 'o' && !event.shiftKey && !event.altKey) {
      event.preventDefault();
      void importLayoutFromFile();
      return;
    }

    if (hasAccel && key === 'c') {
      event.preventDefault();
      copySelectedWidget();
      return;
    }

    if (hasAccel && key === 'v') {
      event.preventDefault();
      pasteCopiedWidget();
      return;
    }

    if (key === 'z' && !event.altKey && hasAccel) {
      event.preventDefault();
      if (event.shiftKey) {
        redoLayout();
      } else {
        undoLayout();
      }
      return;
    }

    if ((event.key === 'Delete' || event.key === 'Backspace') && selectedWidgetId) {
      event.preventDefault();
      removeWidget(selectedWidgetId);
      return;
    }

    if (key === 'i' && (selectedWidgetId !== null || selectedId !== null)) {
      event.preventDefault();
      openInspectorForSelection();
    }
  }

  function onPointerDown(event: PointerEvent) {
    const target = event.target as HTMLElement | null;
    if (target?.closest('.dashboard-context-menu')) return;
    closeDashboardMenu();
  }

  onMount(() => {
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('pointerdown', onPointerDown);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('pointerdown', onPointerDown);
    };
  });

  // Imperative API for the host shell (TopBar, signal rail, grid preferences). Functions that already carry
  // their name are exported directly from the moved block above.
  export function undo() {
    undoLayout();
  }
  export function redo() {
    redoLayout();
  }
  export function createWidgetFromSignal(signal: SignalRow, kind: WidgetKind) {
    createWidget(signal, kind);
  }
  export function requestSignalMap(request: SignalMapRequest) {
    requestSignalMapFromExplorer(request);
  }
  export function setGridColumns(columns: number) {
    handleGridColumnsChange(columns);
  }

  function setPage(tabId: string) {
    layout = { ...layout, activeTabId: tabId };
    selectedWidgetId = null;
    selectedId = null;
    onLayoutChange?.(layout);
  }
</script>

{#if active}
  <section class="canvas-root" aria-label="Dashboard canvas">
    <WorkspaceTabs
      tabs={tabSummaries}
      activeTabId={layout.activeTabId}
      onSelectTab={setPage}
      onAddTab={addTab}
      onRemoveTab={removeTab}
      onRenameTab={renameTab}
    />

    {#if lastError}
      <section class="error-banner panel">{lastError}</section>
    {/if}

    <section class="workspace-stage" aria-label="Dashboard workspace" oncontextmenu={openDashboardMenu}>
      <section class="center-column">
        <WidgetCanvas
          {widgets}
          signals={rows}
          {signalById}
          {selectedWidgetId}
          {widgetInputs}
          historyBySignal={history.values}
          historyTimeBySignal={history.times}
          editMode={editing}
          onSelectWidget={(widgetId, signalId) => {
            selectedWidgetId = widgetId;
            selectedId = signalId && signalId > 0 ? signalId : null;
          }}
          onToggleEditMode={() => (editing = !editing)}
          onClearWidgets={clearWidgets}
          onPatchWidgetLayout={patchWidgetLayout}
          onUpdateWidgetConfig={updateWidgetConfig}
          onRemoveWidget={removeWidget}
          onWidgetInput={setWidgetInput}
          onSendAction={sendAction}
          onSendSet={sendSet}
          onRequestInspector={openInspectorForSelection}
          onDropSignal={(signalId, position, parentLayoutId) => {
            const signal = signalById.get(signalId);
            if (!signal) return;
            selectedId = signalId;
            createWidget(signal, defaultLeafWidgetKind(signal), '', position, parentLayoutId ?? null);
          }}
          onDropTopic={(topicPath, position, parentLayoutId) => {
            const resolved = resolveTopicDrop(topicPath);
            if (!resolved) return;
            selectedId = resolved.signal.signal_id;
            createWidget(resolved.signal, resolved.kind, resolved.title, position, parentLayoutId ?? null);
          }}
          onDropLayoutTool={(kind, position, parentLayoutId) => {
            createLayoutWidget(kind, position, parentLayoutId ?? null);
          }}
          onGridColumnsChange={handleGridColumnsChange}
        />
      </section>

      {#if dashboardMenu}
        <DashboardContextMenu
          style={dashboardMenuStyle(dashboardMenu)}
          canClearLayout={widgets.length > 0}
          canRenamePage={activeTab !== null}
          onImportLayout={importLayoutFromMenu}
          onExportLayout={exportLayoutFromMenu}
          onOpenDataExplorer={openDataExplorerFromMenu}
          onOpenLayoutTools={openLayoutToolsFromMenu}
          onAddPage={addPageFromMenu}
          onRenamePage={renamePageFromMenu}
          onClearLayout={clearLayoutFromMenu}
        />
      {/if}

      {#if showExplorer}
        <FloatingDock
          title={pendingSignalMapRequest ? `Data Explorer - ${pendingSignalMapRequest.title}` : 'Data Explorer'}
          side="left"
          width={370}
          height={560}
          onClose={closeExplorerDock}
        >
          {#if pendingSignalMapRequest}
            <div class="explorer-map-hint panel">
              <strong>{pendingSignalMapRequest.title}</strong>
              <div class="explorer-map-actions">
                {#if pendingSignalMapRequest.allowNone}
                  <button
                    class="btn"
                    onclick={() => {
                      pendingSignalMapRequest?.onPick(null);
                      pendingSignalMapRequest = null;
                      showExplorer = false;
                      query = '';
                    }}
                  >
                    {pendingSignalMapRequest.noneLabel}
                  </button>
                {/if}
                <button
                  class="btn"
                  onclick={() => {
                    pendingSignalMapRequest = null;
                    query = '';
                  }}
                >
                  Cancel mapping
                </button>
              </div>
            </div>
          {/if}
          <SignalBrowser
            signals={explorerSignals}
            ntSignals={filteredNtSignals}
            {selectedId}
            {query}
            onQueryInput={(value) => (query = value)}
            onSelectSignal={(signalId) => {
              if (pendingSignalMapRequest) {
                pendingSignalMapRequest.onPick(signalId);
                selectedId = signalId;
                pendingSignalMapRequest = null;
                showExplorer = false;
                query = '';
                return;
              }
              selectedId = signalId;
              selectedWidgetId = null;
            }}
            onAddSignal={(signal) => {
              if (pendingSignalMapRequest) {
                pendingSignalMapRequest.onPick(signal.signal_id);
                selectedId = signal.signal_id;
                pendingSignalMapRequest = null;
                showExplorer = false;
                query = '';
                return;
              }
              createWidget(signal, defaultWidgetKind(signal));
            }}
            {leafPath}
            showHeader={false}
          />
        </FloatingDock>
      {/if}

      {#if showLayoutTools}
        <FloatingDock title="Layout Tools" side="left" width={320} height={500} onClose={() => (showLayoutTools = false)}>
          <LayoutToolPalette
            query={layoutToolQuery}
            onQueryInput={(value) => (layoutToolQuery = value)}
            onAddLayoutTool={createLayoutWidget}
            showHeader={false}
          />
        </FloatingDock>
      {/if}

      {#if showInspector}
        <FloatingDock title="Inspector" side="right" width={350} height={560} onClose={() => (showInspector = false)}>
          <InspectorPanel
            signals={rows}
            {signalById}
            {selectedSignal}
            {selectedWidget}
            widgetKinds={classicKindsFor(selectedSignal)}
            {widgetKindLabel}
            showHeader={false}
            onShowAsKind={showAsWidgetKind}
            onTriggerAction={sendAction}
            onSelectTunableWidget={() => {
              if (selectedSignal) createWidget(selectedSignal, 'tunable');
            }}
            onRemoveWidget={removeWidget}
            onUpdateWidgetTitle={updateWidgetTitle}
            onUpdateWidgetConfig={updateWidgetConfig}
            onRequestSignalMap={requestSignalMapFromExplorer}
          />
        </FloatingDock>
      {/if}
    </section>

    <button
      class="explorer-fab"
      title={showExplorer ? 'Close Explorer' : 'Open Explorer'}
      aria-label={showExplorer ? 'Close Explorer' : 'Open Explorer'}
      onclick={() => {
        if (showExplorer) {
          closeExplorerDock();
        } else {
          showExplorer = true;
        }
      }}
    >
      <FontAwesomeIcon icon={showExplorer ? faXmark : faMagnifyingGlass} class="explorer-fab-icon" />
    </button>
  </section>
{/if}

<style>
  .canvas-root {
    position: relative;
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
    flex: 1 1 auto;
    min-width: 0;
    min-height: 0;
  }

  .center-column {
    min-width: 0;
    height: 100%;
    display: grid;
    grid-template-rows: 1fr;
    gap: 0.45rem;
    min-height: 0;
  }

  .workspace-stage {
    position: relative;
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    flex: 1 1 auto;
    min-width: 0;
    min-height: 0;
    overflow: hidden;
  }

  .explorer-map-hint {
    padding: 0.44rem;
    border-radius: 8px;
    border: 1px solid rgba(180, 35, 45, 0.38);
    background: rgba(127, 29, 29, 0.16);
    display: grid;
    gap: 0.34rem;
  }

  .explorer-map-hint strong {
    font-size: 0.74rem;
    color: #ffe4e8;
    font-family: var(--font-display);
  }

  .explorer-map-actions {
    display: inline-flex;
    flex-wrap: wrap;
    gap: 0.3rem;
  }

  .explorer-fab {
    position: fixed;
    right: 1rem;
    bottom: 1rem;
    width: 2.9rem;
    height: 2.9rem;
    border-radius: 999px;
    z-index: 80;
    border: 1px solid rgba(180, 35, 45, 0.62);
    background: rgba(180, 35, 45, 0.9);
    color: #fff5f7;
    display: grid;
    place-items: center;
    padding: 0;
    box-shadow: 0 10px 22px rgba(0, 0, 0, 0.34);
  }

  :global(.explorer-fab-icon) {
    width: 1.4rem;
    height: 1.4rem;
    fill: currentColor;
    display: block;
  }
</style>
