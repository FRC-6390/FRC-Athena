<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faMagnifyingGlass, faXmark } from '@fortawesome/free-solid-svg-icons';
  import { onMount } from 'svelte';
  import type { DashboardSnapshot, SignalRow } from './lib/arcp';
  import {
    connectArcp,
    disconnectArcp,
    fireAction,
    listServerLayouts,
    loadServerLayout,
    saveServerLayout,
    setDriverstationDockMode,
    setPresentationMode,
    setSignal,
    snapshotArcp,
    windowModeSnapshot
  } from './lib/arcp';
  import {
    DEFAULT_GRID_COLUMNS,
    applyFilters,
    clampLayout,
    createDefaultLayout,
    createEmptyTab,
    defaultWidgetKind,
    formatCpu,
    formatMemory,
    formatUptime,
    isActionSignal,
    isLayoutWidgetKind,
    leafPath,
    loadLayout,
    makeLayoutWidget,
    makeWidget,
    parseLayoutImport,
    parseNumericSignal,
    saveLayout,
    serializeLayoutExport,
    sparklinePath,
    widgetKindsFor,
    widgetKindLabel,
    type DashboardLayoutState,
    type DashboardWidget,
    type WidgetConfigRecord,
    type LayoutToolKind,
    type WidgetKind,
    type WidgetLayout
  } from './lib/dashboard';
  import { buildDefaultWidgetConfig, cloneWidgetConfig, readLayoutGridConfig } from './lib/widget-config';

  import TopBar from './components/TopBar.svelte';
  import GlobalRail from './components/GlobalRail.svelte';
  import WorkspaceTabs from './components/WorkspaceTabs.svelte';
  import SignalBrowser from './components/SignalBrowser.svelte';
  import LayoutToolPalette from './components/LayoutToolPalette.svelte';
  import WidgetCanvas from './components/WidgetCanvas.svelte';
  import InspectorPanel from './components/InspectorPanel.svelte';
  import SettingsScreen from './components/SettingsScreen.svelte';
  import ActionsScreen from './components/ActionsScreen.svelte';
  import MechanismsScreen from './components/MechanismsScreen.svelte';
  import FloatingDock from './components/FloatingDock.svelte';
  import DashboardContextMenu from './components/DashboardContextMenu.svelte';

  const REFRESH_MS = 75;
  const SERVER_LAYOUT_POLL_MS = 2000;
  const HISTORY_LIMIT = 120;

  type RailSection = 'dashboards' | 'signals' | 'mechanisms' | 'actions' | 'settings';
  type CommitLayoutOptions = {
    recordHistory?: boolean;
    clearRedo?: boolean;
  };
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
  type SignalMapRequest = {
    title: string;
    candidates: SignalRow[];
    selectedSignalId: number | null;
    allowNone: boolean;
    noneLabel: string;
    onPick: (signalId: number | null) => void;
  };

  let host = $state('127.0.0.1');
  let controlPort = $state('5805');
  let serverLayoutName = $state('sim-all-layouts');
  let availableServerLayouts = $state<string[]>([]);
  let connected = $state(false);
  let status = $state('ready');
  let lastError = $state('');

  let snapshot = $state<DashboardSnapshot | null>(null);
  let selectedId = $state<number | null>(null);
  let selectedWidgetId = $state<string | null>(null);

  let layoutState = $state<DashboardLayoutState>(createDefaultLayout());
  let editMode = $state(true);
  let showExplorer = $state(false);
  let showLayoutTools = $state(false);
  let showInspector = $state(false);
  let railSection = $state<RailSection>('dashboards');
  let gridColumns = $state(DEFAULT_GRID_COLUMNS);
  let presentationMode = $state(false);
  let driverstationDockMode = $state(false);

  let widgetInputs = $state<Record<string, string>>({});
  let historyBySignal = $state(new Map<number, number[]>());

  let query = $state('');
  let layoutToolQuery = $state('');
  let roleFilter = $state<string>('all');
  let typeFilter = $state<string>('all');
  let refreshInFlight = false;
  let layoutListRefreshInFlight = false;
  let undoStack = $state<DashboardLayoutState[]>([]);
  let redoStack = $state<DashboardLayoutState[]>([]);
  let copiedWidget = $state<CopiedWidgetSnapshot | null>(null);
  let dashboardMenu = $state<DashboardContextMenuState | null>(null);
  let pendingSignalMapRequest = $state<SignalMapRequest | null>(null);

  const signalRows = $derived(snapshot?.signals ?? []);
  const signalById = $derived(new Map(signalRows.map((signal) => [signal.signal_id, signal])));
  const filteredSignals = $derived(applyFilters(signalRows, query, roleFilter, typeFilter));
  const explorerSignals = $derived.by(() =>
    pendingSignalMapRequest
      ? applyFilters(pendingSignalMapRequest.candidates, query, 'all', 'all')
      : filteredSignals
  );
  const actionSignals = $derived(signalRows.filter((signal) => isActionSignal(signal)));

  const activeTab = $derived(
    layoutState.tabs.find((tab) => tab.id === layoutState.activeTabId) ?? layoutState.tabs[0] ?? null
  );

  const widgets = $derived(activeTab?.widgets ?? []);

  const selectedSignal = $derived(
    selectedId === null ? null : signalById.get(selectedId) ?? null
  );

  const selectedWidget = $derived(
    selectedWidgetId ? widgets.find((widget) => widget.id === selectedWidgetId) ?? null : null
  );

  const tabSummaries = $derived(
    layoutState.tabs.map((tab) => ({
      id: tab.id,
      name: tab.name,
      widgetCount: tab.widgets.length
    }))
  );

  const dashboardStates = $derived([
    { key: 'signals', label: 'signals', value: String(snapshot?.signal_count ?? 0), valueWidthCh: 5 },
    { key: 'updates', label: 'updates', value: String(snapshot?.update_count ?? 0), valueWidthCh: 7 },
    { key: 'uptime', label: 'uptime', value: snapshot ? formatUptime(snapshot.uptime_ms) : '0m 0s', valueWidthCh: 9 },
    { key: 'server-cpu', label: 'server cpu', value: snapshot ? formatCpu(snapshot.server_cpu_percent) : 'n/a', valueWidthCh: 6 },
    { key: 'server-rss', label: 'server rss', value: snapshot ? formatMemory(snapshot.server_rss_bytes) : 'n/a', valueWidthCh: 10 },
    { key: 'ui-cpu', label: 'ui cpu', value: snapshot ? formatCpu(snapshot.host_cpu_percent) : 'n/a', valueWidthCh: 6 },
    { key: 'ui-rss', label: 'ui rss', value: snapshot ? formatMemory(snapshot.host_rss_bytes) : 'n/a', valueWidthCh: 10 }
  ]);

  function activateRail(section: RailSection) {
    dashboardMenu = null;
    railSection = section;
    if (section !== 'dashboards') {
      showLayoutTools = false;
    }
    switch (section) {
      case 'dashboards':
        roleFilter = 'all';
        typeFilter = 'all';
        break;
      case 'signals':
        showInspector = true;
        roleFilter = 'all';
        typeFilter = 'all';
        selectedWidgetId = null;
        break;
      case 'actions':
        roleFilter = 'action';
        typeFilter = 'all';
        query = '';
        showInspector = true;
        break;
      case 'mechanisms':
        roleFilter = 'all';
        typeFilter = 'all';
        query = '';
        showInspector = false;
        selectedWidgetId = null;
        break;
      case 'settings':
        roleFilter = 'all';
        typeFilter = 'all';
        void refreshServerLayoutList();
        break;
      default:
        break;
    }
  }

  async function refreshWindowModes() {
    try {
      const modes = await windowModeSnapshot();
      presentationMode = modes.presentationMode;
      driverstationDockMode = modes.dockMode;
    } catch {
      // ignore if desktop window APIs are unavailable
    }
  }

  async function togglePresentationMode() {
    const enabled = !presentationMode;
    try {
      await setPresentationMode(enabled);
      presentationMode = enabled;
      if (enabled) {
        driverstationDockMode = false;
      }
      status = enabled ? 'presentation mode enabled' : 'presentation mode disabled';
      lastError = '';
    } catch (err) {
      lastError = String(err);
    }
  }

  async function toggleDriverstationDockMode() {
    const enabled = !driverstationDockMode;
    try {
      await setDriverstationDockMode(enabled);
      driverstationDockMode = enabled;
      if (enabled) {
        presentationMode = false;
      }
      status = enabled ? 'driver station dock mode enabled' : 'driver station dock mode disabled';
      lastError = '';
    } catch (err) {
      lastError = String(err);
    }
  }

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

  function commitLayout(next: DashboardLayoutState, options: CommitLayoutOptions = {}) {
    const { recordHistory = true, clearRedo = true } = options;
    if (recordHistory) {
      undoStack = pushHistorySnapshot(undoStack, layoutState);
      if (clearRedo) {
        redoStack = [];
      }
    }

    layoutState = next;
    saveLayout(next);
  }

  function undoLayout() {
    if (undoStack.length === 0) return;

    const previous = undoStack[undoStack.length - 1];
    undoStack = undoStack.slice(0, undoStack.length - 1);
    redoStack = pushHistorySnapshot(redoStack, layoutState);
    layoutState = cloneLayoutState(previous);
    saveLayout(layoutState);

    if (selectedWidgetId && !layoutHasWidget(layoutState, selectedWidgetId)) {
      selectedWidgetId = null;
    }

    status = 'undo';
  }

  function redoLayout() {
    if (redoStack.length === 0) return;

    const next = redoStack[redoStack.length - 1];
    redoStack = redoStack.slice(0, redoStack.length - 1);
    undoStack = pushHistorySnapshot(undoStack, layoutState);
    layoutState = cloneLayoutState(next);
    saveLayout(layoutState);

    if (selectedWidgetId && !layoutHasWidget(layoutState, selectedWidgetId)) {
      selectedWidgetId = null;
    }

    status = 'redo';
  }

  function setActiveWidgets(nextWidgets: DashboardWidget[]) {
    if (!activeTab) return;
    const normalized = normalizeContainerChildren(nextWidgets);
    const nextTabs = layoutState.tabs.map((tab) =>
      tab.id === activeTab.id ? { ...tab, widgets: normalized } : tab
    );
    commitLayout({ ...layoutState, tabs: nextTabs });
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

    const byId = new Map(nextWidgets.map((widget) => [widget.id, widget]));
    const normalized = nextWidgets.map((widget) => ({
      ...widget,
      layout: { ...widget.layout }
    }));

    for (const parent of nextWidgets) {
      if (parent.kind !== 'layout_list') continue;
      const parentSnapshot = normalized.find((widget) => widget.id === parent.id) ?? parent;
      const parentLayout = clampLayout(parentSnapshot.layout, gridColumns);

      const children = normalized
        .filter(
          (widget) =>
            widget.parentLayoutId === parent.id
        )
        .sort(
          (a, b) =>
            a.layout.y - b.layout.y ||
            a.layout.x - b.layout.x ||
            a.id.localeCompare(b.id)
        );

      let cursorY = parentLayout.y;
      for (const child of children) {
        const idx = normalized.findIndex((widget) => widget.id === child.id);
        if (idx < 0) continue;

        const previous = normalized[idx];
        const clampedLayout = clampLayout(
          {
            ...previous.layout,
            x: parentLayout.x,
            y: cursorY,
            w: parentLayout.w
          },
          gridColumns
        );

        normalized[idx] = {
          ...previous,
          parentLayoutId: byId.has(parent.id) ? parent.id : previous.parentLayoutId,
          layout: clampedLayout
        };
        cursorY = clampedLayout.y + clampedLayout.h;
      }

      continue;
    }

    for (const parent of nextWidgets) {
      if (parent.kind !== 'layout_grid') continue;

      const children = normalized
        .filter(
          (widget) =>
            widget.parentLayoutId === parent.id
        )
        .sort(
          (a, b) =>
            a.layout.y - b.layout.y ||
            a.layout.x - b.layout.x ||
            a.id.localeCompare(b.id)
        );

      for (const child of children) {
        const idx = normalized.findIndex((widget) => widget.id === child.id);
        if (idx < 0) continue;

        const previous = normalized[idx];
        normalized[idx] = {
          ...previous,
          parentLayoutId: byId.has(parent.id) ? parent.id : previous.parentLayoutId,
          layout: clampLayout(previous.layout, gridColumns)
        };
      }
    }

    return normalized;
  }

  function updateHistory(signals: SignalRow[]) {
    const next = new Map(historyBySignal);
    const seen = new Set<number>();

    for (const signal of signals) {
      seen.add(signal.signal_id);
      const numeric = parseNumericSignal(signal);
      if (numeric === null) continue;

      const values = [...(next.get(signal.signal_id) ?? [])];
      values.push(numeric);
      if (values.length > 90) {
        values.splice(0, values.length - 90);
      }
      next.set(signal.signal_id, values);
    }

    for (const signalId of next.keys()) {
      if (!seen.has(signalId)) {
        next.delete(signalId);
      }
    }

    historyBySignal = next;
  }

  function createWidget(
    signal: SignalRow,
    kind: WidgetKind,
    titleOverride = '',
    preferredPosition?: Pick<WidgetLayout, 'x' | 'y'>,
    parentLayoutId?: string | null
  ) {
    const config = buildDefaultWidgetConfig(kind, signal, signalRows);
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

    if (parentLayout && parentLayout.kind === 'layout_list') {
      const parentTop = parentLayout.layout.y;
      const parentBottom = parentLayout.layout.y + parentLayout.layout.h;
      const boundedY = Math.max(
        parentTop,
        Math.min(parentBottom - nextWidget.layout.h, nextWidget.layout.y)
      );
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: parentLayout.layout.x,
            y: boundedY,
            w: parentLayout.layout.w
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

    if (parentLayout && parentLayout.kind === 'layout_list') {
      const parentTop = parentLayout.layout.y;
      const parentBottom = parentLayout.layout.y + parentLayout.layout.h;
      const boundedY = Math.max(
        parentTop,
        Math.min(parentBottom - nextWidget.layout.h, nextWidget.layout.y)
      );
      nextWidget = {
        ...nextWidget,
        layout: clampLayout(
          {
            ...nextWidget.layout,
            x: parentLayout.layout.x,
            y: boundedY,
            w: parentLayout.layout.w
          },
          gridColumns
        )
      };
    }

    setActiveWidgets([nextWidget, ...widgets]);
    selectedWidgetId = nextWidget.id;
    selectedId = null;
  }

  function copySelectedWidget(): boolean {
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

    status = `copied ${widget.title}`;
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
    if (railSection !== 'dashboards') return;

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

  function downloadJsonFile(filename: string, contents: string) {
    const blob = new Blob([contents], { type: 'application/json;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const anchor = document.createElement('a');
    anchor.href = url;
    anchor.download = filename;
    anchor.style.display = 'none';
    document.body.appendChild(anchor);
    anchor.click();
    document.body.removeChild(anchor);
    setTimeout(() => URL.revokeObjectURL(url), 0);
  }

  async function pickJsonFileText(): Promise<string | null> {
    return new Promise((resolve, reject) => {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.json,application/json';
      input.style.display = 'none';
      document.body.appendChild(input);

      const cleanup = () => {
        input.removeEventListener('change', onChange);
        document.body.removeChild(input);
      };

      const onChange = async () => {
        try {
          const file = input.files?.[0];
          if (!file) {
            cleanup();
            resolve(null);
            return;
          }
          const text = await file.text();
          cleanup();
          resolve(text);
        } catch (err) {
          cleanup();
          reject(err);
        }
      };

      input.addEventListener('change', onChange, { once: true });
      input.click();
    });
  }

  async function exportLayoutToFile() {
    try {
      const serialized = serializeLayoutExport(layoutState);
      downloadJsonFile(suggestedLayoutFilename(), serialized);
      status = `ui saved (${layoutState.tabs.length} tabs)`;
      lastError = '';
    } catch (err) {
      lastError = `save failed: ${String(err)}`;
    }
  }

  async function importLayoutFromFile() {
    try {
      const raw = await pickJsonFileText();
      if (!raw) return;

      const hasExistingWidgets = layoutState.tabs.some((tab) => tab.widgets.length > 0);
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
      status = `ui loaded (${imported.tabs.length} tabs)`;
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

  function normalizeServerLayoutName(raw: string): string {
    return raw.trim().replace(/[^a-zA-Z0-9._-]/g, '').slice(0, 64);
  }

  async function refreshServerLayoutList() {
    if (!connected) {
      availableServerLayouts = [];
      return;
    }
    if (layoutListRefreshInFlight) return;
    layoutListRefreshInFlight = true;
    try {
      const layouts = await listServerLayouts();
      availableServerLayouts = [...layouts].sort((a, b) =>
        a.localeCompare(b, undefined, { sensitivity: 'base', numeric: true })
      );
      lastError = '';
    } catch (err) {
      availableServerLayouts = [];
      lastError = `server layout list failed: ${String(err)}`;
    } finally {
      layoutListRefreshInFlight = false;
    }
  }

  async function saveLayoutToServerProfile() {
    if (!connected) {
      lastError = 'save to server failed: not connected';
      return;
    }
    const profile = normalizeServerLayoutName(serverLayoutName);
    if (!profile) {
      lastError = 'save to server failed: layout name must be [a-zA-Z0-9._-]';
      return;
    }

    try {
      const serialized = serializeLayoutExport(layoutState);
      await saveServerLayout(profile, serialized);
      serverLayoutName = profile;
      status = `ui saved to server profile '${profile}'`;
      lastError = '';
      await refreshServerLayoutList();
    } catch (err) {
      lastError = `save to server failed: ${String(err)}`;
    }
  }

  async function loadLayoutFromServerProfile() {
    if (!connected) {
      lastError = 'load from server failed: not connected';
      return;
    }
    const profile = normalizeServerLayoutName(serverLayoutName);
    if (!profile) {
      lastError = 'load from server failed: layout name must be [a-zA-Z0-9._-]';
      return;
    }

    try {
      const raw = await loadServerLayout(profile);
      const imported = parseLayoutImport(raw);
      const hasExistingWidgets = layoutState.tabs.some((tab) => tab.widgets.length > 0);
      if (hasExistingWidgets) {
        const confirmed = window.confirm(
          `Load server UI profile '${profile}' and replace current layout? This action can be undone with Ctrl+Z.`
        );
        if (!confirmed) return;
      }

      commitLayout(imported, { recordHistory: true, clearRedo: true });
      selectedWidgetId = null;
      selectedId = null;
      widgetInputs = {};
      status = `ui loaded from server profile '${profile}' (${imported.tabs.length} tabs)`;
      lastError = '';
      serverLayoutName = profile;
    } catch (err) {
      lastError = `load from server failed: ${String(err)}`;
    }
  }

  function pasteCopiedWidget(): boolean {
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
      status = `pasted ${nextWidget.title}`;
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
    status = `pasted ${nextWidget.title}`;
    return true;
  }

  function removeWidget(widgetId: string) {
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

  function updateWidgetConfig(widgetId: string, config: WidgetConfigRecord | undefined) {
    setActiveWidgets(
      widgets.map((widget) =>
        widget.id === widgetId
          ? {
              ...widget,
              config: cloneWidgetConfig(config)
            }
          : widget
      )
    );
  }

  function updateWidgetTitle(widgetId: string, title: string) {
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
    const tab = createEmptyTab(`Dashboard ${layoutState.tabs.length + 1}`);
    commitLayout({
      activeTabId: tab.id,
      tabs: [...layoutState.tabs, tab]
    });
    selectedWidgetId = null;
    selectedId = null;
  }

  function removeTab(tabId: string) {
    if (layoutState.tabs.length <= 1) return;

    const nextTabs = layoutState.tabs.filter((tab) => tab.id !== tabId);
    const nextActive =
      layoutState.activeTabId === tabId ? nextTabs[0].id : layoutState.activeTabId;

    commitLayout({
      activeTabId: nextActive,
      tabs: nextTabs
    });

    selectedWidgetId = null;
  }

  function renameTab(tabId: string, name: string) {
    const nextTabs = layoutState.tabs.map((tab) => (tab.id === tabId ? { ...tab, name } : tab));
    commitLayout({ ...layoutState, tabs: nextTabs });
  }

  async function refreshSnapshot() {
    if (refreshInFlight) return;
    refreshInFlight = true;
    try {
      const next = await snapshotArcp();
      snapshot = next;
      connected = next.connected;
      status = next.status;

      updateHistory(next.signals);

      const validIds = new Set(next.signals.map((signal) => signal.signal_id));
      const nextTabs = layoutState.tabs.map((tab) => {
        const filtered = tab.widgets.filter(
          (widget) => isLayoutWidgetKind(widget.kind) || validIds.has(widget.signalId)
        );
        return filtered.length === tab.widgets.length ? tab : { ...tab, widgets: filtered };
      });

      const changed = nextTabs.some((tab, index) => tab !== layoutState.tabs[index]);
      if (changed) {
        commitLayout({ ...layoutState, tabs: nextTabs }, { recordHistory: false, clearRedo: false });
      }

      if (selectedId !== null && !validIds.has(selectedId)) {
        selectedId = null;
      }

      if (selectedWidgetId && !nextTabs.some((tab) => tab.widgets.some((widget) => widget.id === selectedWidgetId))) {
        selectedWidgetId = null;
      }
    } catch (err) {
      if (!connected) return;
      lastError = String(err);
      connected = false;
    } finally {
      refreshInFlight = false;
    }
  }

  async function connect() {
    lastError = '';
    availableServerLayouts = [];
    try {
      const info = await connectArcp(host.trim(), Number(controlPort));
      connected = info.connected;
      status = `connected to ${info.host}:${info.control_port} (udp ${info.udp_port})`;
      await refreshSnapshot();
      await refreshServerLayoutList();
    } catch (err) {
      connected = false;
      status = 'connection failed';
      lastError = String(err);
      availableServerLayouts = [];
    }
  }

  async function disconnect() {
    try {
      await disconnectArcp();
    } catch (err) {
      lastError = String(err);
    }
    connected = false;
    status = 'disconnected';
    snapshot = null;
    selectedId = null;
    selectedWidgetId = null;
    availableServerLayouts = [];
  }

  async function sendAction(signalId: number) {
    try {
      await fireAction(signalId);
      lastError = '';
      status = `action ${signalId}`;
    } catch (err) {
      lastError = String(err);
    }
  }

  async function sendSet(signalId: number, valueRaw: string) {
    if (!valueRaw.trim()) {
      lastError = 'set value cannot be empty';
      return;
    }

    try {
      await setSignal(signalId, valueRaw);
      lastError = '';
      status = `set ${signalId}`;
    } catch (err) {
      lastError = String(err);
    }
  }

  function handleHostInput(value: string) {
    host = value;
    availableServerLayouts = [];
  }

  function handlePortInput(value: string) {
    controlPort = value;
    availableServerLayouts = [];
  }

  function openInspectorForSelection() {
    if (selectedWidgetId !== null && selectedId === null) {
      const widget = widgets.find((entry) => entry.id === selectedWidgetId);
      if (widget && widget.signalId > 0) {
        selectedId = widget.signalId;
      }
    }
    showInspector = true;
  }

  function shouldIgnoreGlobalHotkeys(target: EventTarget | null): boolean {
    const el = target as HTMLElement | null;
    if (!el) return false;
    const tag = el.tagName.toLowerCase();
    if (tag === 'input' || tag === 'textarea' || tag === 'select' || tag === 'button') {
      return true;
    }
    return el.isContentEditable;
  }

  function onGlobalKeyDown(event: KeyboardEvent) {
    const key = event.key.toLowerCase();
    if (event.key === 'F11') {
      event.preventDefault();
      void togglePresentationMode();
      return;
    }

    const hasAccel = event.ctrlKey || event.metaKey;
    if (hasAccel && event.shiftKey && key === 'd') {
      event.preventDefault();
      void toggleDriverstationDockMode();
      return;
    }

    if (key === 'escape' && (presentationMode || driverstationDockMode)) {
      event.preventDefault();
      if (presentationMode) {
        void setPresentationMode(false).then(
          () => {
            presentationMode = false;
            status = 'presentation mode disabled';
          },
          (err) => {
            lastError = String(err);
          }
        );
      } else {
        void setDriverstationDockMode(false).then(
          () => {
            driverstationDockMode = false;
            status = 'driver station dock mode disabled';
          },
          (err) => {
            lastError = String(err);
          }
        );
      }
      return;
    }

    if (key === 'escape' && dashboardMenu) {
      event.preventDefault();
      closeDashboardMenu();
      return;
    }

    if (event.defaultPrevented || shouldIgnoreGlobalHotkeys(event.target)) {
      return;
    }

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

  onMount(() => {
    layoutState = loadLayout();
    undoStack = [];
    redoStack = [];
    void refreshWindowModes();
    if (host.trim() && Number(controlPort) > 0) {
      void connect();
    }
    const onFullscreenChange = () => {
      void refreshWindowModes();
    };
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target as HTMLElement | null;
      if (target?.closest('.dashboard-context-menu')) return;
      closeDashboardMenu();
    };
    const timer = setInterval(() => {
      void refreshSnapshot();
    }, REFRESH_MS);
    const serverLayoutTimer = setInterval(() => {
      if (!connected || railSection !== 'settings') return;
      void refreshServerLayoutList();
    }, SERVER_LAYOUT_POLL_MS);
    window.addEventListener('keydown', onGlobalKeyDown);
    window.addEventListener('pointerdown', onPointerDown);
    document.addEventListener('fullscreenchange', onFullscreenChange);
    return () => {
      clearInterval(timer);
      clearInterval(serverLayoutTimer);
      window.removeEventListener('keydown', onGlobalKeyDown);
      window.removeEventListener('pointerdown', onPointerDown);
      document.removeEventListener('fullscreenchange', onFullscreenChange);
    };
  });
</script>

<main class="app-shell">
  <GlobalRail section={railSection} onActivate={activateRail} />

  <section class="work-root">
    <TopBar
      {connected}
      {presentationMode}
      {driverstationDockMode}
      {dashboardStates}
      onLoadLayout={() => void importLayoutFromFile()}
      onSaveLayout={() => void exportLayoutToFile()}
      onTogglePresentationMode={() => void togglePresentationMode()}
      onToggleDriverstationDockMode={() => void toggleDriverstationDockMode()}
    />

    <section class="workspace-frame panel">
      {#if railSection === 'dashboards'}
        <WorkspaceTabs
          tabs={tabSummaries}
          activeTabId={layoutState.activeTabId}
          onSelectTab={(tabId) => {
            layoutState = { ...layoutState, activeTabId: tabId };
            selectedWidgetId = null;
            selectedId = null;
            saveLayout(layoutState);
          }}
          onAddTab={addTab}
          onRemoveTab={removeTab}
          onRenameTab={renameTab}
        />
      {/if}

      {#if lastError}
        <section class="error-banner panel">{lastError}</section>
      {/if}

      {#if railSection === 'settings'}
        <SettingsScreen
          {host}
          {controlPort}
          {serverLayoutName}
          availableServerLayouts={availableServerLayouts}
          {connected}
          {status}
          onHostInput={handleHostInput}
          onPortInput={handlePortInput}
          onServerLayoutNameInput={(value) => (serverLayoutName = normalizeServerLayoutName(value))}
          onConnect={connect}
          onDisconnect={disconnect}
          onSaveServerLayout={() => void saveLayoutToServerProfile()}
          onLoadServerLayout={() => void loadLayoutFromServerProfile()}
          onRefreshServerLayouts={() => void refreshServerLayoutList()}
        />
      {:else if railSection === 'signals'}
        <section class={`signals-stage ${showInspector ? '' : 'no-inspector'}`}>
          <div class="signals-layout">
            <SignalBrowser
              signals={filteredSignals}
              {selectedId}
              {query}
              onQueryInput={(value) => (query = value)}
              onSelectSignal={(signalId) => {
                selectedId = signalId;
                selectedWidgetId = null;
              }}
              onAddSignal={(signal) => createWidget(signal, defaultWidgetKind(signal))}
              {leafPath}
            />
            {#if showInspector}
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                onAddWidgetKind={(kind) => {
                  if (selectedSignal) createWidget(selectedSignal, kind);
                }}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) createWidget(selectedSignal, 'tunable');
                }}
                onRemoveWidget={removeWidget}
                onUpdateWidgetTitle={updateWidgetTitle}
                onUpdateWidgetConfig={updateWidgetConfig}
                onRequestSignalMap={requestSignalMapFromExplorer}
              />
            {/if}
          </div>
        </section>
      {:else if railSection === 'actions'}
        <section class={`actions-stage ${showInspector ? '' : 'no-inspector'}`}>
          <div class="actions-layout">
            <ActionsScreen
              actions={actionSignals}
              onTriggerAction={sendAction}
              onSelectSignal={(signalId) => {
                selectedId = signalId;
                showInspector = true;
              }}
            />
            {#if showInspector}
              <InspectorPanel
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                onAddWidgetKind={(kind) => {
                  if (selectedSignal) createWidget(selectedSignal, kind);
                }}
                onTriggerAction={sendAction}
                onSelectTunableWidget={() => {
                  if (selectedSignal) createWidget(selectedSignal, 'tunable');
                }}
                onRemoveWidget={removeWidget}
                onUpdateWidgetTitle={updateWidgetTitle}
                onUpdateWidgetConfig={updateWidgetConfig}
                onRequestSignalMap={requestSignalMapFromExplorer}
              />
            {/if}
          </div>
        </section>
      {:else if railSection === 'mechanisms'}
        <MechanismsScreen
          signals={signalRows}
          {selectedId}
          onSelectSignal={(signalId) => {
            selectedId = signalId;
            selectedWidgetId = null;
          }}
          onTriggerAction={sendAction}
          onSendSet={sendSet}
        />
      {:else}
        <section
          class="workspace-stage"
          aria-label="Dashboard workspace"
          oncontextmenu={openDashboardMenu}
        >
          <section class="center-column">
            <WidgetCanvas
              {widgets}
              signals={signalRows}
              {signalById}
              {selectedWidgetId}
              {widgetInputs}
              {historyBySignal}
              {sparklinePath}
              {editMode}
              onSelectWidget={(widgetId, signalId) => {
                selectedWidgetId = widgetId;
                selectedId = signalId && signalId > 0 ? signalId : null;
              }}
              onToggleEditMode={() => (editMode = !editMode)}
              onClearWidgets={clearWidgets}
              onPatchWidgetLayout={patchWidgetLayout}
              onRemoveWidget={removeWidget}
              onWidgetInput={setWidgetInput}
              onSendAction={sendAction}
              onSendSet={sendSet}
              onRequestInspector={openInspectorForSelection}
              onDropSignal={(signalId, position, parentLayoutId) => {
                const signal = signalById.get(signalId);
                if (!signal) return;
                selectedId = signalId;
                createWidget(signal, defaultWidgetKind(signal), '', position, parentLayoutId ?? null);
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
                signals={signalRows}
                {signalById}
                {selectedSignal}
                {selectedWidget}
                widgetKinds={widgetKindsFor(selectedSignal)}
                {widgetKindLabel}
                showHeader={false}
                onAddWidgetKind={(kind) => {
                  if (selectedSignal) createWidget(selectedSignal, kind);
                }}
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
      {/if}
    </section>
  </section>
</main>

<style>
  .work-root {
    min-width: 0;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.45rem;
    padding: 0.45rem;
  }

  .workspace-frame {
    min-width: 0;
    height: 100%;
    padding: 0.5rem;
    min-height: 0;
    display: flex;
    flex-direction: column;
    gap: 0.45rem;
    overflow: auto;
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

  .explorer-fab-icon {
    width: 1.4rem;
    height: 1.4rem;
    fill: currentColor;
    display: block;
  }

  .signals-stage,
  .actions-stage {
    flex: 1 1 auto;
    min-height: 0;
    display: grid;
    grid-template-rows: 1fr;
    gap: 0;
  }

  .signals-layout,
  .actions-layout {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(280px, 420px) minmax(320px, 1fr);
    gap: 0.58rem;
  }

  .actions-layout {
    grid-template-columns: minmax(320px, 1fr) minmax(300px, 420px);
  }

  .signals-stage.no-inspector .signals-layout,
  .actions-stage.no-inspector .actions-layout {
    grid-template-columns: 1fr;
  }

  @media (max-width: 1200px) {
    .signals-layout,
    .actions-layout {
      grid-template-columns: 1fr;
    }
  }

  @media (max-width: 900px) {
    .signals-stage,
    .actions-stage {
      grid-template-rows: auto 1fr;
    }
  }
</style>
