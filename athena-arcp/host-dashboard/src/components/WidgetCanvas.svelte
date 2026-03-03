<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faClipboard, faUpRightAndDownLeftFromCenter } from '@fortawesome/free-solid-svg-icons';
  import { onMount } from 'svelte';
import BarMeterWidget from './widgets/BarMeterWidget.svelte';
import ButtonGroupWidget from './widgets/ButtonGroupWidget.svelte';
import CameraOverlayWidget from './widgets/CameraOverlayWidget.svelte';
import ControllerEditorWidget from './widgets/ControllerEditorWidget.svelte';
import DialWidget from './widgets/DialWidget.svelte';
import CompassWidget from './widgets/CompassWidget.svelte';
import DifferentialDriveWidget from './widgets/DifferentialDriveWidget.svelte';
import DioWidget from './widgets/DioWidget.svelte';
import DropdownWidget from './widgets/DropdownWidget.svelte';
import EncoderWidget from './widgets/EncoderWidget.svelte';
import FieldViewerWidget from './widgets/FieldViewerWidget.svelte';
import GraphWidget from './widgets/GraphWidget.svelte';
import ImuWidget from './widgets/ImuWidget.svelte';
import Imu3dWidget from './widgets/Imu3dWidget.svelte';
import InputWidget from './widgets/InputWidget.svelte';
import Mech2dWidget from './widgets/Mech2dWidget.svelte';
import MotorWidget from './widgets/MotorWidget.svelte';
import StateMachineWidget from './widgets/StateMachineWidget.svelte';
import RadioWidget from './widgets/RadioWidget.svelte';
import StatusMatrixWidget from './widgets/StatusMatrixWidget.svelte';
import SwerveDriveWidget from './widgets/SwerveDriveWidget.svelte';
import SwerveModuleWidget from './widgets/SwerveModuleWidget.svelte';
import TextAreaWidget from './widgets/TextAreaWidget.svelte';
import TimerWidget from './widgets/TimerWidget.svelte';
import ToggleWidget from './widgets/ToggleWidget.svelte';
  import type { SignalRow } from '../lib/arcp';
  import type {
    DashboardWidget,
    LayoutToolKind,
    WidgetConfigRecord,
    WidgetLayout
  } from '../lib/dashboard';
  import { readLayoutAccordionConfig, readLayoutGridConfig, readLayoutTitleConfig } from '../lib/widget-config';
  import {
    DEFAULT_GRID_COLUMNS,
    clampLayout,
    isActionSignal,
    isLayoutWidgetKind,
    layoutsOverlap,
    widgetKindLabel
  } from '../lib/dashboard';

  type Props = {
    widgets: DashboardWidget[];
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    selectedWidgetId: string | null;
    widgetInputs: Record<string, string>;
    historyBySignal: Map<number, number[]>;
    historyTimeBySignal: Map<number, number[]>;
    sparklinePath: (values: number[]) => string;
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
    sparklinePath,
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

  const GRID_GAP = 6;
  const MIN_ROWS = 8;
  const MIN_GRID_COLUMNS = DEFAULT_GRID_COLUMNS;
  const MIN_CELL_SIZE = 22;
  const MAX_GRID_COLUMNS = 96;
  const SCROLLBAR_SAFETY_GUTTER = 18;
  const SIGNAL_DRAG_TYPE = 'application/x-arcp-signal-id';
  const TOPIC_DRAG_TYPE = 'application/x-arcp-topic-path';
  const LAYOUT_DRAG_TYPE = 'application/x-arcp-layout-kind';
  const DEFAULT_SIGNAL_DROP_SIZE = { w: 2, h: 1 };

  type DragMode = 'move' | 'resize';
  type DragState = {
    widgetId: string;
    mode: DragMode;
    startX: number;
    startY: number;
    pointerX: number;
    pointerY: number;
    startGridX: number;
    startGridY: number;
    anchorOffsetX: number;
    anchorOffsetY: number;
    resizeOffsetX: number;
    resizeOffsetY: number;
    frozenLayoutBounds: Map<string, WidgetLayout>;
    startLayout: WidgetLayout;
    currentLayout: WidgetLayout;
  };

  type ContextMenuState = {
    widgetId: string;
    signalId: number | null;
    x: number;
    y: number;
  };

  let gridEl = $state<HTMLDivElement | null>(null);
  let gridWidth = $state(860);
  let gridHeight = $state(620);
  let drag = $state<DragState | null>(null);
  let contextMenu = $state<ContextMenuState | null>(null);
  let dragOverSurface = $state(false);
  let copyToastVisible = $state(false);
  let copyToastTimer: ReturnType<typeof setTimeout> | null = null;
  let layoutMeasureTick = $state(0);

  const widgetById = $derived.by(() => new Map(widgets.map((widget) => [widget.id, widget])));

  const maxWidgetColumn = $derived.by(() => {
    let maxColumn = DEFAULT_GRID_COLUMNS;
    for (const widget of widgets) {
      const right = widget.layout.x + widget.layout.w;
      if (right > maxColumn) {
        maxColumn = right;
      }
    }
    return maxColumn;
  });

  const gridColumns = $derived.by(() => {
    const resolved = Math.ceil(Math.max(DEFAULT_GRID_COLUMNS, maxWidgetColumn));
    return Math.max(MIN_GRID_COLUMNS, Math.min(MAX_GRID_COLUMNS, resolved));
  });

  const cellSize = $derived(
    Math.max(MIN_CELL_SIZE, (gridWidth - (gridColumns - 1) * GRID_GAP) / gridColumns)
  );

  const visibleRows = $derived(
    Math.max(1, Math.floor((gridHeight + GRID_GAP) / (cellSize + GRID_GAP)))
  );

  const canvasContentWidth = $derived(gridColumns * cellSize + (gridColumns - 1) * GRID_GAP);

  const canvasRows = $derived.by(() => {
    let maxRow = MIN_ROWS;
    for (const widget of widgets) {
      const bottom = widget.layout.y + widget.layout.h;
      if (bottom > maxRow) maxRow = bottom;
    }
    return Math.ceil(Math.max(maxRow, visibleRows));
  });

  const canvasContentHeight = $derived(canvasRows * cellSize + (canvasRows - 1) * GRID_GAP);

  const widgetsForRender = $derived.by(() => {
    const siblingCompare = (a: DashboardWidget, b: DashboardWidget): number => {
      const aLayout = isLayoutWidgetKind(a.kind);
      const bLayout = isLayoutWidgetKind(b.kind);
      if (aLayout !== bLayout) return aLayout ? -1 : 1;
      return (
        a.layout.y - b.layout.y ||
        a.layout.x - b.layout.x ||
        a.id.localeCompare(b.id)
      );
    };

    const childrenByParent = new Map<string | null, DashboardWidget[]>();
    for (const widget of widgets) {
      const parentKey =
        widget.parentLayoutId && widgetById.has(widget.parentLayoutId) ? widget.parentLayoutId : null;
      const children = childrenByParent.get(parentKey);
      if (children) {
        children.push(widget);
      } else {
        childrenByParent.set(parentKey, [widget]);
      }
    }

    for (const children of childrenByParent.values()) {
      children.sort(siblingCompare);
    }

    const ordered: DashboardWidget[] = [];
    const visited = new Set<string>();
    const visit = (widget: DashboardWidget) => {
      if (visited.has(widget.id)) return;
      visited.add(widget.id);
      ordered.push(widget);
      const children = childrenByParent.get(widget.id) ?? [];
      for (const child of children) {
        visit(child);
      }
    };

    for (const root of childrenByParent.get(null) ?? []) {
      visit(root);
    }

    if (ordered.length < widgets.length) {
      const fallback = [...widgets].sort(siblingCompare);
      for (const widget of fallback) {
        visit(widget);
      }
    }

    return ordered;
  });

  const accordionCollapsedById = $derived.by(() => {
    const map = new Map<string, boolean>();
    for (const widget of widgets) {
      if (widget.kind === 'layout_accordion') {
        map.set(widget.id, readLayoutAccordionConfig(widget.config).collapsed);
      }
    }
    return map;
  });

  const hiddenByCollapsedAccordionByWidgetId = $derived.by(() => {
    const hidden = new Map<string, boolean>();
    for (const widget of widgets) {
      let isHidden = false;
      let cursor = widget.parentLayoutId;
      const seen = new Set<string>();
      while (cursor) {
        if (seen.has(cursor)) break;
        seen.add(cursor);
        if (accordionCollapsedById.get(cursor)) {
          isHidden = true;
          break;
        }
        cursor = widgetById.get(cursor)?.parentLayoutId ?? null;
      }
      hidden.set(widget.id, isHidden);
    }
    return hidden;
  });

  const resolvedSignalByWidgetId = $derived.by(() => {
    const map = new Map<string, SignalRow | null>();
    for (const widget of widgets) {
      map.set(widget.id, resolveWidgetSignal(widget));
    }
    return map;
  });

  const layoutContentBoundsById = $derived.by(() => {
    layoutMeasureTick;
    gridEl;
    cellSize;
    gridColumns;
    const map = new Map<string, WidgetLayout>();
    for (const widget of widgets) {
      if (!isLayoutWidgetKind(widget.kind)) continue;
      map.set(widget.id, resolveLayoutContentBoundsLive(widget));
    }
    return map;
  });

  const draggingWidget = $derived.by(() => (drag ? widgetById.get(drag.widgetId) ?? null : null));

  const childWidgetsByContainerId = $derived.by(() => {
    drag;
    const byParent = new Map<string, DashboardWidget[]>();
    for (const widget of widgets) {
      const parentId = widget.parentLayoutId;
      if (!parentId) continue;
      const parent = widgetById.get(parentId);
      if (!parent || !isContainerLayoutKind(parent.kind)) continue;
      const siblings = byParent.get(parentId);
      if (siblings) {
        siblings.push(widget);
      } else {
        byParent.set(parentId, [widget]);
      }
    }
    for (const siblings of byParent.values()) {
      siblings.sort((a, b) => {
        const aLayout =
          drag && drag.widgetId === a.id ? drag.currentLayout : clampLayout(a.layout, gridColumns);
        const bLayout =
          drag && drag.widgetId === b.id ? drag.currentLayout : clampLayout(b.layout, gridColumns);
        return aLayout.y - bLayout.y || aLayout.x - bLayout.x || a.id.localeCompare(b.id);
      });
    }
    return byParent;
  });

  const containerNormalizedLayoutByWidgetId = $derived.by(() => {
    drag;
    const normalized = new Map<string, WidgetLayout>();

    for (const parent of widgets) {
      if (!isContainerLayoutKind(parent.kind)) continue;
      const children = childWidgetsByContainerId.get(parent.id) ?? [];
      if (children.length === 0) continue;
      const parentLayout =
        parent.kind === 'layout_grid'
          ? (resolveGridLayoutSpec(parent)?.parent ?? resolveLayoutContentBounds(parent))
          : resolveLayoutContentBounds(parent);

      if (parent.kind === 'layout_accordion') {
        for (const child of children) {
          const base =
            drag && drag.widgetId === child.id ? drag.currentLayout : clampLayout(child.layout, gridColumns);
          normalized.set(
            child.id,
            clampLayout(
              {
                ...base,
                x: parentLayout.x,
                y: parentLayout.y,
                w: parentLayout.w,
                h: parentLayout.h
              },
              gridColumns
            )
          );
        }
        continue;
      }

      if (parent.kind === 'layout_list') {
        let cursorY = parentLayout.y;
        for (const child of children) {
          const base =
            drag && drag.widgetId === child.id ? drag.currentLayout : clampLayout(child.layout, gridColumns);
          const childLayout = clampLayout(
            {
              ...base,
              x: parentLayout.x,
              y: cursorY,
              w: parentLayout.w
            },
            gridColumns
          );
          normalized.set(child.id, childLayout);
          cursorY = childLayout.y + childLayout.h;
        }
        continue;
      }

      if (parent.kind === 'layout_grid') {
        for (const child of children) {
          const base =
            drag && drag.widgetId === child.id ? drag.currentLayout : clampLayout(child.layout, gridColumns);
          const cellLayout = gridCellLayoutFromAbsolute(parent, base);
          if (!cellLayout) {
            normalized.set(child.id, base);
            continue;
          }
          normalized.set(child.id, absoluteLayoutFromGridCell(parent, cellLayout) ?? base);
        }
      }
    }

    return normalized;
  });

  const renderedContentBounds = $derived.by(() => {
    let maxRightPx = 0;
    let maxBottomPx = 0;

    for (const widget of widgetsForRender) {
      if (isHiddenByCollapsedAccordion(widget)) continue;

      const layout = resolvedWidgetLayout(widget);
      const rect = computeWidgetPixelRect(layout, Boolean(widget.parentLayoutId));
      const right = rect.left + Math.max(1, rect.width);
      const bottom = rect.top + Math.max(1, rect.height);
      if (right > maxRightPx) maxRightPx = right;
      if (bottom > maxBottomPx) maxBottomPx = bottom;
    }

    return {
      maxRightPx,
      maxBottomPx
    };
  });

  const canvasEffectiveWidth = $derived(
    Math.max(canvasContentWidth, Math.ceil(renderedContentBounds.maxRightPx))
  );
  const canvasEffectiveHeight = $derived(
    Math.max(canvasContentHeight, Math.ceil(renderedContentBounds.maxBottomPx))
  );

  const canvasOverflowX = $derived(canvasEffectiveWidth > gridWidth);
  const canvasOverflowY = $derived(canvasEffectiveHeight > gridHeight);

  const canvasScrollWidth = $derived(
    canvasEffectiveWidth + (canvasOverflowY ? SCROLLBAR_SAFETY_GUTTER : 0)
  );
  const canvasScrollHeight = $derived(
    canvasEffectiveHeight + (canvasOverflowX ? SCROLLBAR_SAFETY_GUTTER : 0)
  );

  function normalizeSignalPathForLookup(rawPath: string): string {
    return rawPath
      .trim()
      .replace(/\\/g, '/')
      .replace(/\/+/g, '/')
      .replace(/^\/+|\/+$/g, '')
      .toLowerCase();
  }

  function canonicalSignalToken(value: string): string {
    return value.toLowerCase().replace(/[^a-z0-9]/g, '');
  }

  function canonicalSignalPathForLookup(rawPath: string): string {
    return rawPath
      .trim()
      .replace(/\\/g, '/')
      .replace(/\/+/g, '/')
      .replace(/^\/+|\/+$/g, '')
      .split('/')
      .map((segment) => canonicalSignalToken(segment))
      .filter((segment) => segment.length > 0)
      .join('/');
  }

  function isPlaceholderSignalValue(row: SignalRow): boolean {
    const value = row.value.trim();
    return value.length === 0 || value === '-';
  }

  function resolveSignalFromCanonicalTopic(topicPath: string, preferredLeafPath = ''): SignalRow | null {
    const canonicalTopic = canonicalSignalPathForLookup(topicPath);
    if (!canonicalTopic) return null;
    const preferredLeaf = canonicalSignalToken(
      preferredLeafPath.slice(preferredLeafPath.lastIndexOf('/') + 1)
    );

    const candidates: SignalRow[] = [];
    for (const entry of signals) {
      const canonicalPath = canonicalSignalPathForLookup(entry.path);
      if (!canonicalPath) continue;
      if (
        canonicalPath === canonicalTopic ||
        canonicalPath.startsWith(`${canonicalTopic}/`)
      ) {
        candidates.push(entry);
      }
    }
    if (candidates.length === 0) return null;

    const scored = candidates
      .map((entry) => {
        const canonicalLeaf = canonicalSignalToken(
          entry.path.slice(entry.path.lastIndexOf('/') + 1)
        );
        let score = 0;
        if (preferredLeaf && canonicalLeaf === preferredLeaf) score += 8;
        if (!isPlaceholderSignalValue(entry)) score += 4;
        if (entry.access === 'observe') score += 1;
        return { entry, score };
      })
      .sort((a, b) => b.score - a.score || a.entry.path.localeCompare(b.entry.path));

    return scored[0]?.entry ?? null;
  }

  const signalByNormalizedPath = $derived.by(() => {
    const map = new Map<string, SignalRow>();
    for (const entry of signals) {
      const normalized = normalizeSignalPathForLookup(entry.path);
      if (!normalized || map.has(normalized)) continue;
      map.set(normalized, entry);
    }
    return map;
  });

  function resolveSignalFromPath(rawPath: string, topicPath: string): SignalRow | null {
    const cleanedRaw = rawPath.trim();
    if (!cleanedRaw) return null;

    const normalizedTopic = normalizeSignalPathForLookup(topicPath);
    let candidate = cleanedRaw;
    if (candidate.startsWith('./')) {
      candidate = candidate.slice(2);
    }

    let resolvedPath = '';
    if (candidate.startsWith('/')) {
      resolvedPath = normalizeSignalPathForLookup(candidate.slice(1));
    } else {
      const normalizedCandidate = normalizeSignalPathForLookup(candidate);
      if (!normalizedCandidate) return null;

      const hasQualifiedRoot =
        normalizedCandidate.startsWith('athena/') ||
        normalizedCandidate.startsWith('robot/');
      resolvedPath =
        hasQualifiedRoot || !normalizedTopic
          ? normalizedCandidate
          : normalizeSignalPathForLookup(`${normalizedTopic}/${normalizedCandidate}`);
    }

    if (!resolvedPath) return null;
    const exact = signalByNormalizedPath.get(resolvedPath);
    if (exact) return exact;

    const canonicalResolved = canonicalSignalPathForLookup(resolvedPath);
    if (canonicalResolved) {
      for (const entry of signals) {
        if (canonicalSignalPathForLookup(entry.path) === canonicalResolved) {
          return entry;
        }
      }
    }

    if (!candidate.includes('/')) {
      const normalizedLeaf = normalizeSignalPathForLookup(candidate);
      if (!normalizedLeaf) return null;
      const canonicalTopic = canonicalSignalPathForLookup(normalizedTopic);
      for (const entry of signals) {
        const normalizedPath = normalizeSignalPathForLookup(entry.path);
        if (!normalizedPath) continue;
        if (normalizedTopic) {
          const inNormalizedScope =
            normalizedPath === normalizedTopic ||
            normalizedPath.startsWith(`${normalizedTopic}/`);
          const canonicalPath = canonicalSignalPathForLookup(entry.path);
          const inCanonicalScope =
            canonicalTopic.length > 0 &&
            (canonicalPath === canonicalTopic ||
              canonicalPath.startsWith(`${canonicalTopic}/`));
          if (!inNormalizedScope && !inCanonicalScope) continue;
        }
        const leaf = normalizedPath.slice(normalizedPath.lastIndexOf('/') + 1);
        if (leaf === normalizedLeaf || leaf.endsWith(normalizedLeaf)) {
          return entry;
        }
      }
    }

    return null;
  }

  function collectConfiguredSignalIds(value: unknown, out: Set<number>): void {
    if (value === null || value === undefined) return;
    if (typeof value === 'number') {
      if (Number.isFinite(value) && value > 0) {
        out.add(Math.floor(value));
      }
      return;
    }
    if (Array.isArray(value)) {
      for (const item of value) {
        collectConfiguredSignalIds(item, out);
      }
      return;
    }
    if (typeof value !== 'object') {
      return;
    }
    const record = value as Record<string, unknown>;
    for (const [key, nested] of Object.entries(record)) {
      if (key.toLowerCase().endsWith('signalid')) {
        collectConfiguredSignalIds(nested, out);
        continue;
      }
      if (
        key === 'series' ||
        key === 'actions' ||
        key === 'columns' ||
        key === 'signals' ||
        key === 'targets'
      ) {
        collectConfiguredSignalIds(nested, out);
      }
    }
  }

  function collectConfiguredSignalPaths(value: unknown, out: Set<string>): void {
    if (value === null || value === undefined) return;
    if (typeof value === 'string') {
      const trimmed = value.trim();
      if (trimmed) out.add(trimmed);
      return;
    }
    if (Array.isArray(value)) {
      for (const item of value) {
        collectConfiguredSignalPaths(item, out);
      }
      return;
    }
    if (typeof value !== 'object') {
      return;
    }
    const record = value as Record<string, unknown>;
    for (const [key, nested] of Object.entries(record)) {
      const lower = key.toLowerCase();
      if (lower.endsWith('signalpath') || key === 'signalPath') {
        collectConfiguredSignalPaths(nested, out);
        continue;
      }
      if (
        key === 'series' ||
        key === 'actions' ||
        key === 'columns' ||
        key === 'signals' ||
        key === 'targets'
      ) {
        collectConfiguredSignalPaths(nested, out);
      }
    }
  }

  function resolveWidgetSignal(widget: DashboardWidget): SignalRow | null {
    const directSignal =
      Number.isFinite(widget.signalId) && widget.signalId > 0
        ? signalById.get(widget.signalId) ?? null
        : null;
    const config = widget.config;
    if (!config || typeof config !== 'object' || Array.isArray(config)) {
      return directSignal;
    }

    const topicPath =
      typeof config.topicPath === 'string' ? config.topicPath.trim() : '';
    const normalizedTopic = normalizeSignalPathForLookup(topicPath);

    const configuredSignalPath =
      typeof config.signalPath === 'string' ? config.signalPath.trim() : '';
    if (configuredSignalPath) {
      const resolved = resolveSignalFromPath(configuredSignalPath, topicPath);
      if (resolved) return resolved;
    }

    if (directSignal) {
      if (!normalizedTopic) {
        return directSignal;
      }
      const normalizedDirect = normalizeSignalPathForLookup(directSignal.path);
      if (
        normalizedDirect === normalizedTopic ||
        normalizedDirect.startsWith(`${normalizedTopic}/`)
      ) {
        if (!isPlaceholderSignalValue(directSignal)) {
          return directSignal;
        }
        const canonicalFallback = resolveSignalFromCanonicalTopic(topicPath, directSignal.path);
        if (canonicalFallback && canonicalFallback.signal_id !== directSignal.signal_id) {
          return canonicalFallback;
        }
        return directSignal;
      }
    }

    const candidateIds = new Set<number>();
    collectConfiguredSignalIds(config, candidateIds);
    let bestIdCandidate: SignalRow | null = null;
    for (const id of candidateIds) {
      const resolved = signalById.get(id);
      if (!resolved) continue;
      if (!bestIdCandidate || !isPlaceholderSignalValue(resolved)) {
        bestIdCandidate = resolved;
      }
      if (!isPlaceholderSignalValue(resolved)) break;
    }
    if (bestIdCandidate) return bestIdCandidate;

    const candidatePaths = new Set<string>();
    collectConfiguredSignalPaths(config, candidatePaths);
    for (const rawPath of candidatePaths) {
      const resolved = resolveSignalFromPath(rawPath, topicPath);
      if (resolved) return resolved;
    }

    if (normalizedTopic) {
      const exact = signalByNormalizedPath.get(normalizedTopic);
      if (exact && !isPlaceholderSignalValue(exact)) return exact;
      for (const entry of signals) {
        const normalizedPath = normalizeSignalPathForLookup(entry.path);
        if (normalizedPath.startsWith(`${normalizedTopic}/`)) {
          if (!isPlaceholderSignalValue(entry)) return entry;
          if (!exact) return entry;
        }
      }
      const canonicalFallback = resolveSignalFromCanonicalTopic(
        topicPath,
        directSignal?.path ?? ''
      );
      if (canonicalFallback) return canonicalFallback;
      if (exact) return exact;
    }

    if (directSignal) {
      return directSignal;
    }
    return null;
  }

  function unavailableSignalHint(widget: DashboardWidget): string {
    if (widget.signalId > 0) {
      return `signal #${widget.signalId}`;
    }
    const config = widget.config;
    if (!config || typeof config !== 'object' || Array.isArray(config)) {
      return 'no bound signal';
    }
    const topicPath = typeof config.topicPath === 'string' ? config.topicPath.trim() : '';
    if (topicPath) {
      return topicPath;
    }
    const signalPath = typeof config.signalPath === 'string' ? config.signalPath.trim() : '';
    if (signalPath) {
      return signalPath;
    }
    return 'no bound signal';
  }

  function computeWidgetPixelRect(
    layout: WidgetLayout,
    withoutTrailingGap: boolean
  ): { left: number; top: number; width: number; height: number } {
    const step = cellSize + GRID_GAP;
    const left = (layout.x - 1) * step;
    const top = (layout.y - 1) * step;
    const width = withoutTrailingGap
      ? layout.w * step
      : layout.w >= 1
        ? layout.w * step - GRID_GAP
        : layout.w * step;
    const height = withoutTrailingGap
      ? layout.h * step
      : layout.h >= 1
        ? layout.h * step - GRID_GAP
        : layout.h * step;
    return { left, top, width, height };
  }

  function isDescendantOf(widget: DashboardWidget, ancestorId: string): boolean {
    let cursor = widget.parentLayoutId;
    while (cursor) {
      if (cursor === ancestorId) return true;
      const parent = widgetById.get(cursor);
      cursor = parent?.parentLayoutId;
    }
    return false;
  }

  function isAccordionCollapsed(widget: DashboardWidget | null | undefined): boolean {
    if (!widget || widget.kind !== 'layout_accordion') return false;
    return readLayoutAccordionConfig(widget.config).collapsed;
  }

  function isHiddenByCollapsedAccordion(widget: DashboardWidget): boolean {
    return hiddenByCollapsedAccordionByWidgetId.get(widget.id) ?? false;
  }

  function accordionHasOtherChildren(accordionId: string, ignoreWidgetId?: string): boolean {
    return widgets.some(
      (widget) =>
        widget.parentLayoutId === accordionId &&
        (ignoreWidgetId === undefined || widget.id !== ignoreWidgetId)
    );
  }

  function toggleAccordionCollapsed(widget: DashboardWidget): void {
    if (widget.kind !== 'layout_accordion') return;
    const baseConfig =
      widget.config && typeof widget.config === 'object' && !Array.isArray(widget.config)
        ? widget.config
        : {};
    const nextCollapsed = !readLayoutAccordionConfig(widget.config).collapsed;
    onUpdateWidgetConfig(widget.id, {
      ...(baseConfig as WidgetConfigRecord),
      collapsed: nextCollapsed
    });
  }

  function accordionStackLayer(widget: DashboardWidget): number {
    let layer = 0;
    let first = true;
    let cursor: DashboardWidget | null = widget;
    while (cursor) {
      if (cursor.kind === 'layout_accordion' && !isAccordionCollapsed(cursor)) {
        // Accordion shell floats above baseline; descendants render above shell.
        layer += first ? 2 : 3;
      }
      first = false;
      const parentId = cursor.parentLayoutId;
      cursor = parentId ? widgetById.get(parentId) ?? null : null;
    }
    return layer;
  }

  function widgetStyle(
    layout: WidgetLayout,
    withoutTrailingGap = false,
    zIndex?: number
  ): string {
    const box = computeWidgetPixelRect(layout, withoutTrailingGap);
    const z = zIndex !== undefined ? `z-index:${zIndex};` : '';
    return `transform:translate3d(${box.left}px,${box.top}px,0);width:${Math.max(1, box.width)}px;height:${Math.max(1, box.height)}px;${z}`;
  }

  function widgetStyleFor(widget: DashboardWidget, layout: WidgetLayout): string {
    const withoutTrailingGap = Boolean(widget.parentLayoutId);
    const stackLayer = accordionStackLayer(widget);
    const stackedZIndex = stackLayer > 0 ? 10 + stackLayer : undefined;
    if (!drag || drag.mode !== 'move') {
      return widgetStyle(layout, withoutTrailingGap, stackedZIndex);
    }

    const pointerDx = drag.pointerX - drag.startX;
    const pointerDy = drag.pointerY - drag.startY;
    const movingWidget = draggingWidget;

    if (widget.id !== drag.widgetId) {
      // When a layout container moves, render its descendants with the same live drag offset.
      if (
        movingWidget &&
        isLayoutWidgetKind(movingWidget.kind) &&
        isDescendantOf(widget, movingWidget.id)
      ) {
        const box = computeWidgetPixelRect(layout, true);
        const left = box.left + pointerDx;
        const top = box.top + pointerDy;
        return `transform:translate3d(${left}px,${top}px,0);width:${Math.max(1, box.width)}px;height:${Math.max(1, box.height)}px;z-index:22;`;
      }

      return widgetStyle(layout, withoutTrailingGap, stackedZIndex);
    }

    const startBox = computeWidgetPixelRect(drag.startLayout, withoutTrailingGap);
    const currentBox = computeWidgetPixelRect(layout, withoutTrailingGap);
    const baseLeft = startBox.left;
    const baseTop = startBox.top;
    const left = baseLeft + pointerDx;
    const top = baseTop + pointerDy;
    const draggingLayoutShell = isLayoutWidgetKind(widget.kind);
    const zIndex = draggingLayoutShell ? 16 : 20;
    const opacity = draggingLayoutShell ? 'opacity:0.88;' : '';
    return `transform:translate3d(${left}px,${top}px,0);width:${Math.max(1, currentBox.width)}px;height:${Math.max(1, currentBox.height)}px;z-index:${zIndex};${opacity}`;
  }

  function layoutsEqual(a: WidgetLayout, b: WidgetLayout): boolean {
    return a.x === b.x && a.y === b.y && a.w === b.w && a.h === b.h;
  }

  function resolvedWidgetLayout(widget: DashboardWidget): WidgetLayout {
    const base = clampLayout(widget.layout, gridColumns);
    if (drag && drag.widgetId === widget.id) {
      return drag.currentLayout;
    }
    return containerNormalizedLayoutByWidgetId.get(widget.id) ?? base;
  }

  type GridLayoutSpec = {
    parent: WidgetLayout;
    columns: number;
    rows: number;
    xEdges: number[];
    yEdges: number[];
  };

  function rawWidgetLayout(widget: DashboardWidget): WidgetLayout {
    return clampLayout(widget.layout, gridColumns);
  }

  function escapedWidgetId(id: string): string {
    if (typeof CSS !== 'undefined' && typeof CSS.escape === 'function') {
      return CSS.escape(id);
    }
    return id.replace(/"/g, '\\"');
  }

  function pointerToCanvasUnits(clientX: number, clientY: number): Pick<WidgetLayout, 'x' | 'y'> | null {
    if (!gridEl) return null;
    const rect = gridEl.getBoundingClientRect();
    const step = cellSize + GRID_GAP;
    if (step <= 0) return null;
    return {
      x: (clientX - rect.left + gridEl.scrollLeft) / step + 1,
      y: (clientY - rect.top + gridEl.scrollTop) / step + 1
    };
  }

  function resolveLayoutContentElement(layoutWidget: DashboardWidget): HTMLElement | null {
    if (!gridEl) return null;
    const rootSelector = `.widget-card[data-widget-id="${escapedWidgetId(layoutWidget.id)}"]`;
    const rootEl = gridEl.querySelector(rootSelector) as HTMLElement | null;
    if (!rootEl) return null;
    if (layoutWidget.kind === 'layout_grid') {
      return (rootEl.querySelector('.layout-grid') as HTMLElement | null) ?? rootEl;
    }
    if (layoutWidget.kind === 'layout_list') {
      return (rootEl.querySelector('.layout-list') as HTMLElement | null) ?? rootEl;
    }
    if (layoutWidget.kind === 'layout_accordion') {
      return (rootEl.querySelector('.layout-accordion-body') as HTMLElement | null) ?? rootEl;
    }
    return (rootEl.querySelector('.widget-body') as HTMLElement | null) ?? rootEl;
  }

  function resolveLayoutContentBoundsLive(layoutWidget: DashboardWidget): WidgetLayout {
    const fallback = rawWidgetLayout(layoutWidget);
    if (!gridEl) return fallback;
    const contentEl = resolveLayoutContentElement(layoutWidget);
    if (!contentEl) return fallback;

    const gridRect = gridEl.getBoundingClientRect();
    const contentRect = contentEl.getBoundingClientRect();
    const step = cellSize + GRID_GAP;
    const computedStyle = window.getComputedStyle(contentEl);
    const borderLeft = Number.parseFloat(computedStyle.borderLeftWidth) || 0;
    const borderTop = Number.parseFloat(computedStyle.borderTopWidth) || 0;
    const borderRight = Number.parseFloat(computedStyle.borderRightWidth) || 0;
    const borderBottom = Number.parseFloat(computedStyle.borderBottomWidth) || 0;
    const innerLeftPx =
      contentRect.left - gridRect.left + gridEl.scrollLeft + borderLeft;
    const innerTopPx =
      contentRect.top - gridRect.top + gridEl.scrollTop + borderTop;
    const innerWidthPx = Math.max(0, contentRect.width - borderLeft - borderRight);
    const innerHeightPx = Math.max(0, contentRect.height - borderTop - borderBottom);
    const x = innerLeftPx / step + 1;
    const y = innerTopPx / step + 1;
    const w = Math.max(1e-3, innerWidthPx / step);
    const h = Math.max(1e-3, innerHeightPx / step);

    const measured = clampLayout({ x, y, w, h }, gridColumns);

    if (isContainerLayoutKind(layoutWidget.kind)) {
      const anchor = normalizeContainerAxisForBounds(layoutWidget, fallback);
      // Keep nested container children aligned to the parent's true grid X/width.
      // Using measured inner X/width compounds visual inset at every nesting level.
      return clampLayout(
        {
          ...measured,
          x: anchor.x,
          w: anchor.w
        },
        gridColumns
      );
    }

    return measured;
  }

  function normalizeContainerAxisForBounds(
    widget: DashboardWidget,
    fallback: WidgetLayout
  ): WidgetLayout {
    let normalized = fallback;
    normalized = normalizedAccordionChildLayout(widget, normalized) ?? normalized;
    normalized = normalizedListChildLayout(widget, normalized) ?? normalized;
    normalized = normalizedGridChildLayout(widget, normalized) ?? normalized;
    return normalized;
  }

  function snapshotLayoutBoundsForDrag(): Map<string, WidgetLayout> {
    const bounds = new Map<string, WidgetLayout>();
    for (const widget of widgets) {
      if (!isLayoutWidgetKind(widget.kind)) continue;
      bounds.set(widget.id, resolveLayoutContentBoundsLive(widget));
    }
    return bounds;
  }

  function resolveLayoutContentBounds(layoutWidget: DashboardWidget): WidgetLayout {
    const frozen = drag?.frozenLayoutBounds.get(layoutWidget.id);
    if (frozen) return frozen;
    return layoutContentBoundsById.get(layoutWidget.id) ?? resolveLayoutContentBoundsLive(layoutWidget);
  }

  function buildGridAxisEdges(start: number, span: number, segments: number): number[] {
    const count = Math.max(1, Math.floor(segments));
    const edges: number[] = [];
    for (let index = 0; index <= count; index++) {
      edges.push(start + (index * span) / count);
    }
    return edges;
  }

  function resolveGridLayoutSpec(layoutWidget: DashboardWidget): GridLayoutSpec | null {
    if (layoutWidget.kind !== 'layout_grid') return null;
    const parent = resolveLayoutContentBounds(layoutWidget);
    const config = readLayoutGridConfig(
      layoutWidget.config,
      Math.max(1, Math.floor(parent.w)),
      Math.max(1, Math.floor(parent.h))
    );
    const columns = config.autoSize
      ? Math.max(1, Math.min(96, Math.round(parent.w)))
      : Math.max(1, Math.floor(config.columns));
    const rows = config.autoSize
      ? Math.max(1, Math.min(96, Math.round(parent.h)))
      : Math.max(1, Math.floor(config.rows));
    return {
      parent,
      columns,
      rows,
      xEdges: buildGridAxisEdges(parent.x, parent.w, columns),
      yEdges: buildGridAxisEdges(parent.y, parent.h, rows)
    };
  }

  function axisCellAtValue(value: number, edges: number[]): number {
    const count = Math.max(1, edges.length - 1);
    if (value <= edges[0]) return 1;
    if (value >= edges[count]) return count;

    let low = 0;
    let high = count - 1;
    while (low <= high) {
      const mid = Math.floor((low + high) / 2);
      const cellStart = edges[mid];
      const cellEnd = edges[mid + 1];
      if (value < cellStart) {
        high = mid - 1;
      } else if (value >= cellEnd) {
        low = mid + 1;
      } else {
        return mid + 1;
      }
    }
    return Math.max(1, Math.min(count, low + 1));
  }

  function axisCellFromRange(
    rangeStart: number,
    rangeEnd: number,
    edges: number[]
  ): { index: number; span: number } {
    const count = Math.max(1, edges.length - 1);
    const axisMin = edges[0];
    const axisMax = edges[count];
    const start = Math.max(axisMin, Math.min(axisMax, Math.min(rangeStart, rangeEnd)));
    const end = Math.max(axisMin, Math.min(axisMax, Math.max(rangeStart, rangeEnd)));
    const axisSpan = Math.max(1e-6, axisMax - axisMin);
    const averageCellSpan = axisSpan / count;
    const epsilon = Math.max(1e-6, averageCellSpan * 0.01);

    const interiorStart = Math.max(axisMin, Math.min(axisMax, start + epsilon));
    const interiorEnd = Math.max(axisMin, Math.min(axisMax, end - epsilon));
    const sampleStart = interiorStart <= interiorEnd ? interiorStart : (start + end) * 0.5;
    const sampleEnd = interiorStart <= interiorEnd ? interiorEnd : sampleStart;
    const first = axisCellAtValue(sampleStart, edges);
    const last = axisCellAtValue(sampleEnd, edges);

    if (first > last) {
      const clamped = Math.max(1, Math.min(count, first));
      return { index: clamped, span: 1 };
    }

    return {
      index: first,
      span: Math.max(1, last - first + 1)
    };
  }

  function axisRangeFromCell(
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

  function gridCellLayoutFromAbsolute(
    layoutWidget: DashboardWidget,
    layout: WidgetLayout
  ): WidgetLayout | null {
    const spec = resolveGridLayoutSpec(layoutWidget);
    if (!spec) return null;

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

    const col = axisCellFromRange(xStart, xEnd, spec.xEdges);
    const row = axisCellFromRange(yStart, yEnd, spec.yEdges);
    return {
      x: col.index,
      y: row.index,
      w: Math.max(1, Math.min(spec.columns - col.index + 1, col.span)),
      h: Math.max(1, Math.min(spec.rows - row.index + 1, row.span))
    };
  }

  function absoluteLayoutFromGridCell(
    layoutWidget: DashboardWidget,
    cellLayout: WidgetLayout
  ): WidgetLayout | null {
    const spec = resolveGridLayoutSpec(layoutWidget);
    if (!spec) return null;
    const xRange = axisRangeFromCell(cellLayout.x, cellLayout.w, spec.xEdges);
    const yRange = axisRangeFromCell(cellLayout.y, cellLayout.h, spec.yEdges);
    const maxX = spec.parent.x + spec.parent.w;
    const maxY = spec.parent.y + spec.parent.h;
    const x = Math.max(spec.parent.x, Math.min(maxX, xRange.start));
    const y = Math.max(spec.parent.y, Math.min(maxY, yRange.start));
    const w = Math.max(1e-6, Math.min(maxX - x, xRange.span));
    const h = Math.max(1e-6, Math.min(maxY - y, yRange.span));
    return clampLayout(
      {
        x,
        y,
        w,
        h
      },
      gridColumns
    );
  }

  function normalizedAccordionChildLayout(
    widget: DashboardWidget,
    fallback: WidgetLayout
  ): WidgetLayout | null {
    if (!widget.parentLayoutId) return null;

    const parent = widgetById.get(widget.parentLayoutId);
    if (!parent || parent.kind !== 'layout_accordion') return null;

    const parentLayout = resolveLayoutContentBounds(parent);
    return clampLayout(
      {
        ...fallback,
        x: parentLayout.x,
        y: parentLayout.y,
        w: parentLayout.w,
        h: parentLayout.h
      },
      gridColumns
    );
  }

  function normalizedListChildLayout(
    widget: DashboardWidget,
    fallback: WidgetLayout
  ): WidgetLayout | null {
    if (!widget.parentLayoutId) return null;

    const parent = widgetById.get(widget.parentLayoutId);
    if (!parent || parent.kind !== 'layout_list') return null;

    const parentLayout = resolveLayoutContentBounds(parent);
    const siblings = widgets
      .filter(
        (entry) =>
          entry.parentLayoutId === parent.id
      )
      .map((entry) => ({
        id: entry.id,
        layout:
          drag && drag.widgetId === entry.id
            ? drag.currentLayout
            : clampLayout(entry.layout, gridColumns)
      }))
      .sort(
        (a, b) =>
          a.layout.y - b.layout.y ||
          a.layout.x - b.layout.x ||
          a.id.localeCompare(b.id)
      );

    let cursorY = parentLayout.y;
    for (const sibling of siblings) {
      const normalized = clampLayout(
        {
          ...sibling.layout,
          x: parentLayout.x,
          y: cursorY,
          w: parentLayout.w
        },
        gridColumns
      );
      if (sibling.id === widget.id) {
        return normalized;
      }
      cursorY = normalized.y + normalized.h;
    }

    return fallback;
  }

  function normalizedGridChildLayout(
    widget: DashboardWidget,
    fallback: WidgetLayout
  ): WidgetLayout | null {
    if (!widget.parentLayoutId) return null;

    const parent = widgetById.get(widget.parentLayoutId);
    if (!parent || parent.kind !== 'layout_grid') return null;

    const cellLayout = gridCellLayoutFromAbsolute(parent, fallback);
    if (!cellLayout) return fallback;
    return absoluteLayoutFromGridCell(parent, cellLayout) ?? fallback;
  }

  function hasCollision(widgetId: string, candidate: WidgetLayout): boolean {
    const movingWidget = widgetById.get(widgetId);
    if (!movingWidget || isLayoutWidgetKind(movingWidget.kind) || isHiddenByCollapsedAccordion(movingWidget)) {
      return false;
    }

    return widgets.some((widget) => {
      if (widget.id === widgetId) return false;
      if (isHiddenByCollapsedAccordion(widget)) return false;
      if (isLayoutWidgetKind(widget.kind)) return false;
      return layoutsOverlap(resolvedWidgetLayout(widget), candidate);
    });
  }

  function isContainerLayoutKind(kind: DashboardWidget['kind']): boolean {
    return kind === 'layout_list' || kind === 'layout_grid' || kind === 'layout_accordion';
  }

  function findContainerParent(widget: DashboardWidget): DashboardWidget | null {
    const parentId = widget.parentLayoutId;
    if (!parentId) return null;
    const parent = widgetById.get(parentId);
    if (!parent || !isContainerLayoutKind(parent.kind)) return null;
    return parent;
  }

  function layoutContainsPoint(layout: WidgetLayout, x: number, y: number): boolean {
    return x >= layout.x && x < layout.x + layout.w && y >= layout.y && y < layout.y + layout.h;
  }

  function wouldCreateParentCycle(widgetId: string, candidateParentId: string): boolean {
    let cursor: string | undefined = candidateParentId;
    while (cursor) {
      if (cursor === widgetId) return true;
      const node = widgetById.get(cursor);
      cursor = node?.parentLayoutId;
    }
    return false;
  }

  function snapMoveWithinContainer(
    moving: DashboardWidget,
    candidate: WidgetLayout,
    startLayout: WidgetLayout
  ): WidgetLayout {
    if (isLayoutWidgetKind(moving.kind)) {
      return candidate;
    }
    const parent = findContainerParent(moving);
    if (!parent) {
      return candidate;
    }
    const parentLayout =
      parent.kind === 'layout_grid'
        ? (resolveGridLayoutSpec(parent)?.parent ?? resolveLayoutContentBounds(parent))
        : resolveLayoutContentBounds(parent);
    const candidateCenterX = candidate.x + candidate.w * 0.5;
    const candidateCenterY = candidate.y + candidate.h * 0.5;

    // Let widgets leave a container when their center is dragged outside.
    if (!layoutContainsPoint(parentLayout, candidateCenterX, candidateCenterY)) {
      return candidate;
    }

    if (parent.kind === 'layout_accordion') {
      return clampLayout(
        {
          ...candidate,
          x: parentLayout.x,
          y: parentLayout.y,
          w: parentLayout.w,
          h: parentLayout.h
        },
        gridColumns
      );
    }

    if (parent.kind === 'layout_list') {
      const snappedPos = resolveLayoutDropPosition(
        parent,
        { x: candidate.x, y: candidate.y },
        { w: candidate.w, h: candidate.h },
        moving.id
      );
      return clampLayout(
        {
          ...candidate,
          x: parentLayout.x,
          y: snappedPos.y,
          w: parentLayout.w
        },
        gridColumns
      );
    }

    const spec = resolveGridLayoutSpec(parent);
    if (!spec) return candidate;
    const startCell = gridCellLayoutFromAbsolute(parent, startLayout) ?? { x: 1, y: 1, w: 1, h: 1 };
    const candidateCell = gridCellLayoutFromAbsolute(parent, candidate);
    if (!candidateCell) {
      return candidate;
    }

    const snappedCell: WidgetLayout = {
      x: Math.max(1, Math.min(Math.max(1, spec.columns - startCell.w + 1), candidateCell.x)),
      y: Math.max(1, Math.min(Math.max(1, spec.rows - startCell.h + 1), candidateCell.y)),
      w: startCell.w,
      h: startCell.h
    };
    const absolute = absoluteLayoutFromGridCell(parent, snappedCell);
    return absolute ?? candidate;
  }

  function snapLayoutToCanvasCells(layout: WidgetLayout): WidgetLayout {
    return clampLayout(
      {
        ...layout,
        x: Math.round(layout.x),
        y: Math.round(layout.y),
        w: Math.max(1, Math.round(layout.w)),
        h: Math.max(1, Math.round(layout.h))
      },
      gridColumns
    );
  }

  function snapLayoutToExistingParentGrid(
    moving: DashboardWidget,
    layout: WidgetLayout,
    startLayout: WidgetLayout,
    mode: DragMode
  ): WidgetLayout {
    const parent = findContainerParent(moving);
    if (parent?.kind !== 'layout_grid') return layout;
    const spec = resolveGridLayoutSpec(parent);
    const startCell = gridCellLayoutFromAbsolute(parent, startLayout);
    const candidateCell = gridCellLayoutFromAbsolute(parent, layout);
    if (!spec || !startCell || !candidateCell) return layout;

    if (mode === 'move') {
      const maxCol = Math.max(1, spec.columns - startCell.w + 1);
      const maxRow = Math.max(1, spec.rows - startCell.h + 1);
      const snapped = absoluteLayoutFromGridCell(parent, {
        x: Math.max(1, Math.min(maxCol, candidateCell.x)),
        y: Math.max(1, Math.min(maxRow, candidateCell.y)),
        w: startCell.w,
        h: startCell.h
      });
      return snapped ?? layout;
    }

    const maxW = Math.max(1, spec.columns - startCell.x + 1);
    const maxH = Math.max(1, spec.rows - startCell.y + 1);
    const snapped = absoluteLayoutFromGridCell(parent, {
      x: startCell.x,
      y: startCell.y,
      w: Math.max(1, Math.min(maxW, candidateCell.w)),
      h: Math.max(1, Math.min(maxH, candidateCell.h))
    });
    return snapped ?? layout;
  }

  function snapLayoutToExistingParentList(
    moving: DashboardWidget,
    layout: WidgetLayout,
    startLayout: WidgetLayout,
    mode: DragMode
  ): WidgetLayout {
    const parent = findContainerParent(moving);
    if (parent?.kind !== 'layout_list' && parent?.kind !== 'layout_accordion') return layout;
    const parentLayout = resolveLayoutContentBounds(parent);

    if (parent.kind === 'layout_accordion') {
      if (mode === 'move') {
        const centerX = layout.x + layout.w * 0.5;
        const centerY = layout.y + layout.h * 0.5;
        if (!layoutContainsPoint(parentLayout, centerX, centerY)) {
          return layout;
        }
      }
      return clampLayout(
        {
          ...layout,
          x: parentLayout.x,
          y: parentLayout.y,
          w: parentLayout.w,
          h: parentLayout.h
        },
        gridColumns
      );
    }

    if (mode === 'move') {
      const centerX = layout.x + layout.w * 0.5;
      const centerY = layout.y + layout.h * 0.5;
      if (!layoutContainsPoint(parentLayout, centerX, centerY)) {
        return layout;
      }
      const snappedPos = resolveLayoutDropPosition(
        parent,
        { x: layout.x, y: layout.y },
        { w: layout.w, h: layout.h },
        moving.id
      );
      const maxY = Math.max(parentLayout.y, parentLayout.y + parentLayout.h - layout.h);
      return clampLayout(
        {
          ...layout,
          x: parentLayout.x,
          y: Math.max(parentLayout.y, Math.min(maxY, snappedPos.y)),
          w: parentLayout.w
        },
        gridColumns
      );
    }

    const maxHeight = Math.max(1, parentLayout.y + parentLayout.h - startLayout.y);
    const snappedHeight = Math.max(1, Math.round(layout.h));
    return clampLayout(
      {
        ...layout,
        x: parentLayout.x,
        y: startLayout.y,
        w: parentLayout.w,
        h: Math.max(1, Math.min(maxHeight, snappedHeight))
      },
      gridColumns
    );
  }

  function resolveResizeDragWidget(
    widget: DashboardWidget,
    parentContainer: DashboardWidget | null
  ): DashboardWidget {
    if (parentContainer?.kind === 'layout_accordion' && !isAccordionCollapsed(parentContainer)) {
      return parentContainer;
    }
    return widget;
  }

  function beginDrag(event: PointerEvent, widget: DashboardWidget, mode: DragMode) {
    if (!editMode) return;
    if (event.button !== 0) return;

    closeContextMenu();

    let dragWidget = widget;
    if (mode === 'resize') {
      const parent = findContainerParent(widget);
      if (parent?.kind === 'layout_accordion' && !isAccordionCollapsed(parent)) {
        // Resize gestures on accordion contents should resize the accordion shell itself.
        dragWidget = parent;
      }
    }

    const clampedStart = resolvedWidgetLayout(dragWidget);
    const pointerOnCanvas = pointerToCanvasUnits(event.clientX, event.clientY);
    const startGridX = pointerOnCanvas?.x ?? clampedStart.x;
    const startGridY = pointerOnCanvas?.y ?? clampedStart.y;
    const anchorOffsetX = startGridX - clampedStart.x;
    const anchorOffsetY = startGridY - clampedStart.y;
    const resizeOffsetX = clampedStart.x + clampedStart.w - startGridX;
    const resizeOffsetY = clampedStart.y + clampedStart.h - startGridY;

    drag = {
      widgetId: dragWidget.id,
      mode,
      startX: event.clientX,
      startY: event.clientY,
      pointerX: event.clientX,
      pointerY: event.clientY,
      startGridX,
      startGridY,
      anchorOffsetX,
      anchorOffsetY,
      resizeOffsetX,
      resizeOffsetY,
      frozenLayoutBounds: snapshotLayoutBoundsForDrag(),
      startLayout: clampedStart,
      currentLayout: clampedStart
    };

    event.preventDefault();
    event.stopPropagation();
  }

  function applyDragAt(pointerX: number, pointerY: number) {
    if (!drag) return;

    const moving = widgetById.get(drag.widgetId);
    if (!moving) {
      drag = null;
      return;
    }

    const pointerOnCanvas = pointerToCanvasUnits(pointerX, pointerY);
    const baseUnitsPerCell = cellSize + GRID_GAP;
    const pointerGridX =
      pointerOnCanvas?.x ?? drag.startGridX + (pointerX - drag.startX) / Math.max(1e-6, baseUnitsPerCell);
    const pointerGridY =
      pointerOnCanvas?.y ?? drag.startGridY + (pointerY - drag.startY) / Math.max(1e-6, baseUnitsPerCell);
    const freeMoveCandidate: WidgetLayout = {
      ...drag.startLayout,
      x: pointerGridX - drag.anchorOffsetX,
      y: pointerGridY - drag.anchorOffsetY
    };
    let freeResizeCandidate: WidgetLayout = {
      ...drag.startLayout,
      w: Math.max(1e-3, pointerGridX + drag.resizeOffsetX - drag.startLayout.x),
      h: Math.max(1e-3, pointerGridY + drag.resizeOffsetY - drag.startLayout.y)
    };
    if (drag.mode === 'resize' && moving.kind === 'layout_accordion' && isAccordionCollapsed(moving)) {
      // Collapsed accordions stay one row tall; resize gestures should affect width.
      freeResizeCandidate = {
        ...freeResizeCandidate,
        h: Math.max(1, Math.round(drag.startLayout.h))
      };
    }

    let next: WidgetLayout;
    const containerParent = findContainerParent(moving);
    if (containerParent?.kind === 'layout_grid') {
      const spec = resolveGridLayoutSpec(containerParent);
      const startCell = gridCellLayoutFromAbsolute(containerParent, drag.startLayout);
      if (spec && startCell) {
        if (drag.mode === 'move') {
          const parentLayout = spec.parent;
          const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
          const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
          if (!layoutContainsPoint(parentLayout, centerX, centerY)) {
            next = freeMoveCandidate;
          } else {
            const candidateCell = gridCellLayoutFromAbsolute(containerParent, freeMoveCandidate);
            if (!candidateCell) {
              next = drag.startLayout;
            } else {
              const maxCol = Math.max(1, spec.columns - startCell.w + 1);
              const maxRow = Math.max(1, spec.rows - startCell.h + 1);
              const targetCell: WidgetLayout = {
                x: Math.max(1, Math.min(maxCol, candidateCell.x)),
                y: Math.max(1, Math.min(maxRow, candidateCell.y)),
                w: startCell.w,
                h: startCell.h
              };
              next = absoluteLayoutFromGridCell(containerParent, targetCell) ?? drag.startLayout;
            }
          }
        } else {
          const candidateCell = gridCellLayoutFromAbsolute(containerParent, freeResizeCandidate);
          if (!candidateCell) {
            next = drag.startLayout;
          } else {
            const targetCell: WidgetLayout = {
              x: startCell.x,
              y: startCell.y,
              w: startCell.w,
              h: startCell.h
            };
            const maxW = Math.max(1, spec.columns - startCell.x + 1);
            const maxH = Math.max(1, spec.rows - startCell.y + 1);
            targetCell.w = Math.max(1, Math.min(maxW, candidateCell.w));
            targetCell.h = Math.max(1, Math.min(maxH, candidateCell.h));
            next = absoluteLayoutFromGridCell(containerParent, targetCell) ?? drag.startLayout;
          }
        }
      } else {
        next = drag.mode === 'move' ? freeMoveCandidate : freeResizeCandidate;
      }
    } else if (containerParent?.kind === 'layout_accordion') {
      const parentLayout = resolveLayoutContentBounds(containerParent);
      if (drag.mode === 'move') {
        const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
        const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
        if (!layoutContainsPoint(parentLayout, centerX, centerY)) {
          next = freeMoveCandidate;
        } else {
          next = clampLayout(
            {
              ...drag.startLayout,
              x: parentLayout.x,
              y: parentLayout.y,
              w: parentLayout.w,
              h: parentLayout.h
            },
            gridColumns
          );
        }
      } else {
        next = clampLayout(
          {
            ...drag.startLayout,
            x: parentLayout.x,
            y: parentLayout.y,
            w: parentLayout.w,
            h: parentLayout.h
          },
          gridColumns
        );
      }
    } else if (containerParent?.kind === 'layout_list') {
      const parentLayout = resolveLayoutContentBounds(containerParent);
      if (drag.mode === 'move') {
        const centerX = freeMoveCandidate.x + freeMoveCandidate.w * 0.5;
        const centerY = freeMoveCandidate.y + freeMoveCandidate.h * 0.5;
        if (!layoutContainsPoint(parentLayout, centerX, centerY)) {
          next = freeMoveCandidate;
        } else {
          const snappedPos = resolveLayoutDropPosition(
            containerParent,
            { x: freeMoveCandidate.x, y: freeMoveCandidate.y },
            { w: drag.startLayout.w, h: drag.startLayout.h },
            moving.id
          );
          const maxY = Math.max(parentLayout.y, parentLayout.y + parentLayout.h - drag.startLayout.h);
          next = clampLayout(
            {
              ...drag.startLayout,
              x: parentLayout.x,
              y: Math.max(parentLayout.y, Math.min(maxY, snappedPos.y)),
              w: parentLayout.w
            },
            gridColumns
          );
        }
      } else {
        const maxHeight = Math.max(1, parentLayout.y + parentLayout.h - drag.startLayout.y);
        const snappedHeight = Math.max(1, Math.round(freeResizeCandidate.h));
        next = clampLayout(
          {
            ...drag.startLayout,
            x: parentLayout.x,
            y: drag.startLayout.y,
            w: parentLayout.w,
            h: Math.max(1, Math.min(maxHeight, snappedHeight))
          },
          gridColumns
        );
      }
    } else if (drag.mode === 'move') {
      next = freeMoveCandidate;
      next = snapMoveWithinContainer(moving, next, drag.startLayout);
      if (!containerParent) {
        next = snapLayoutToCanvasCells(next);
      }
    } else {
      next = freeResizeCandidate;
      if (!containerParent) {
        next = snapLayoutToCanvasCells(next);
      }
    }

    const clamped = clampLayout(next, gridColumns);
    const pointerChanged = pointerX !== drag.pointerX || pointerY !== drag.pointerY;

    if (hasCollision(drag.widgetId, clamped)) {
      if (pointerChanged) {
        drag = {
          ...drag,
          pointerX,
          pointerY
        };
      }
      return;
    }

    if (layoutsEqual(clamped, drag.currentLayout) && !pointerChanged) {
      return;
    }

    drag = {
      ...drag,
      pointerX,
      pointerY,
      currentLayout: clamped
    };
  }

  function applyDrag(event: PointerEvent) {
    applyDragAt(event.clientX, event.clientY);
  }

  function endDrag() {
    if (!drag) return;

    const { widgetId, startLayout, currentLayout, mode } = drag;
    const moving = widgetById.get(widgetId);
    if (!moving) {
      drag = null;
      return;
    }

    let finalLayout = currentLayout;
    let parentLayoutId: string | null | undefined = undefined;

    if (mode === 'move') {
      const layoutTarget = findLayoutWidgetAtCell(
        {
          x: finalLayout.x + finalLayout.w * 0.5,
          y: finalLayout.y + finalLayout.h * 0.5
        },
        widgetId,
        isLayoutWidgetKind(moving.kind) ? moving.id : undefined
      );
      const targetAcceptsDrop =
        layoutTarget?.kind !== 'layout_accordion' ||
        !accordionHasOtherChildren(layoutTarget.id, widgetId) ||
        layoutTarget.id === moving.parentLayoutId;
      if (
        layoutTarget &&
        targetAcceptsDrop &&
        isContainerLayoutKind(layoutTarget.kind) &&
        !wouldCreateParentCycle(widgetId, layoutTarget.id)
      ) {
        const snappedPos = resolveLayoutDropPosition(
          layoutTarget,
          { x: finalLayout.x, y: finalLayout.y },
          { w: finalLayout.w, h: finalLayout.h },
          widgetId
        );
        const adjustedForLayout =
          layoutTarget.kind === 'layout_accordion'
            ? (() => {
              const targetLayout = resolveLayoutContentBounds(layoutTarget);
              return {
                ...finalLayout,
                x: targetLayout.x,
                y: targetLayout.y,
                w: targetLayout.w,
                h: targetLayout.h
              };
            })()
            : layoutTarget.kind === 'layout_list'
              ? (() => {
                const targetLayout = resolveLayoutContentBounds(layoutTarget);
                return {
                  ...finalLayout,
                  ...snappedPos,
                  x: targetLayout.x,
                  w: targetLayout.w
                };
              })()
              : (() => {
                const sourceGridParent = findContainerParent(moving);
                const movingWithinSameGrid =
                  sourceGridParent?.kind === 'layout_grid' && sourceGridParent.id === layoutTarget.id;
                const startCell =
                  gridCellLayoutFromAbsolute(layoutTarget, startLayout) ?? { x: 1, y: 1, w: 1, h: 1 };
                const dropAnchorCell =
                  gridCellLayoutFromAbsolute(layoutTarget, {
                    x: snappedPos.x,
                    y: snappedPos.y,
                    w: 1,
                    h: 1
                  }) ??
                  gridCellLayoutFromAbsolute(layoutTarget, {
                    ...finalLayout,
                    ...snappedPos
                  });
                const sourceSizeCell = gridCellLayoutFromAbsolute(layoutTarget, {
                  ...finalLayout,
                  x: finalLayout.x,
                  y: finalLayout.y
                });
                if (!dropAnchorCell) {
                  return { ...finalLayout, ...snappedPos };
                }
                const spec = resolveGridLayoutSpec(layoutTarget);
                if (!spec) {
                  return { ...finalLayout, ...snappedPos };
                }
                const desiredW = movingWithinSameGrid ? startCell.w : (sourceSizeCell?.w ?? 1);
                const desiredH = movingWithinSameGrid ? startCell.h : (sourceSizeCell?.h ?? 1);
                const maxW = Math.max(1, spec.columns - dropAnchorCell.x + 1);
                const maxH = Math.max(1, spec.rows - dropAnchorCell.y + 1);
                const snappedCell: WidgetLayout = {
                  x: dropAnchorCell.x,
                  y: dropAnchorCell.y,
                  w: Math.max(1, Math.min(maxW, desiredW)),
                  h: Math.max(1, Math.min(maxH, desiredH))
                };
                return (
                  absoluteLayoutFromGridCell(layoutTarget, snappedCell) ?? {
                    ...finalLayout,
                    ...snappedPos
                  }
                );
              })();
        const snapped = clampLayout(adjustedForLayout, gridColumns);
        if (!hasCollision(widgetId, snapped)) {
          finalLayout = snapped;
          parentLayoutId = layoutTarget.id;
        }
      } else if (!layoutTarget) {
        parentLayoutId = null;
      }
    }

    // If parent did not change, force a final snap to current parent grid.
    if (parentLayoutId === undefined) {
      const sourceParent = findContainerParent(moving);
      if (sourceParent?.kind === 'layout_list' || sourceParent?.kind === 'layout_accordion') {
        finalLayout = snapLayoutToExistingParentList(moving, finalLayout, startLayout, mode);
      } else if (sourceParent?.kind === 'layout_grid') {
        finalLayout = snapLayoutToExistingParentGrid(moving, finalLayout, startLayout, mode);
      }
    }
    const effectiveParentId = parentLayoutId === undefined ? moving.parentLayoutId ?? null : parentLayoutId;
    const effectiveParent = effectiveParentId
      ? widgetById.get(effectiveParentId) ?? null
      : null;
    if (!effectiveParent) {
      finalLayout = snapLayoutToCanvasCells(finalLayout);
    }

    const parentChanged =
      parentLayoutId !== undefined && (moving.parentLayoutId ?? null) !== (parentLayoutId ?? null);
    if (!layoutsEqual(startLayout, finalLayout) || parentChanged) {
      onPatchWidgetLayout(widgetId, finalLayout, parentLayoutId);
    }

    drag = null;
  }

  function closeContextMenu() {
    contextMenu = null;
  }

  function menuStyle(menu: ContextMenuState): string {
    const viewportWidth = typeof window === 'undefined' ? 1920 : window.innerWidth;
    const viewportHeight = typeof window === 'undefined' ? 1080 : window.innerHeight;
    const x = Math.min(menu.x, viewportWidth - 220);
    const y = Math.min(menu.y, viewportHeight - 120);
    return `left:${Math.max(8, x)}px;top:${Math.max(8, y)}px;`;
  }

  function openContextMenu(event: MouseEvent, widget: DashboardWidget, signalId: number | null) {
    event.preventDefault();
    event.stopPropagation();
    onSelectWidget(widget.id, signalId);
    contextMenu = {
      widgetId: widget.id,
      signalId,
      x: event.clientX,
      y: event.clientY
    };
  }

  function openInspector(event?: Event) {
    event?.preventDefault();
    event?.stopPropagation();
    onRequestInspector();
    closeContextMenu();
  }

  async function copySignalValue(signalId: number) {
    const signal = signalById.get(signalId);
    if (!signal) return;
    const value = signal.value ?? '';

    try {
      await navigator.clipboard.writeText(value);
      showCopyToast();
      return;
    } catch {
      // Fallback for environments where clipboard API is unavailable.
    }

    const input = document.createElement('textarea');
    input.value = value;
    input.setAttribute('readonly', 'true');
    input.style.position = 'fixed';
    input.style.opacity = '0';
    input.style.pointerEvents = 'none';
    document.body.appendChild(input);
    input.select();
    try {
      const copied = document.execCommand('copy');
      if (copied) {
        showCopyToast();
      }
    } finally {
      document.body.removeChild(input);
    }
  }

  function showCopyToast() {
    copyToastVisible = true;
    if (copyToastTimer) {
      clearTimeout(copyToastTimer);
    }
    copyToastTimer = setTimeout(() => {
      copyToastVisible = false;
      copyToastTimer = null;
    }, 1100);
  }

  function findLayoutWidgetAtCell(
    cell: Pick<WidgetLayout, 'x' | 'y'>,
    ignoreWidgetId?: string,
    ignoreDescendantsOfId?: string
  ): DashboardWidget | null {
    const candidates = widgets.filter((widget) => {
      if (widget.id === ignoreWidgetId) return false;
      if (!isLayoutWidgetKind(widget.kind)) return false;
      if (ignoreDescendantsOfId && isDescendantOf(widget, ignoreDescendantsOfId)) return false;
      if (isHiddenByCollapsedAccordion(widget)) return false;
      if (isAccordionCollapsed(widget)) return false;
      const layout = resolvedWidgetLayout(widget);
      const xEnd = layout.x + layout.w;
      const yEnd = layout.y + layout.h;
      return cell.x >= layout.x && cell.x < xEnd && cell.y >= layout.y && cell.y < yEnd;
    });

    if (candidates.length === 0) return null;

    const byId = new Map(widgets.map((widget) => [widget.id, widget]));
    const depthCache = new Map<string, number>();
    const depthVisiting = new Set<string>();
    const hierarchyDepth = (widgetId: string): number => {
      const cached = depthCache.get(widgetId);
      if (cached !== undefined) return cached;
      if (depthVisiting.has(widgetId)) return 0;
      depthVisiting.add(widgetId);
      const widget = byId.get(widgetId);
      const parentId = widget?.parentLayoutId;
      let depth = 0;
      if (parentId && byId.has(parentId)) {
        depth = hierarchyDepth(parentId) + 1;
      }
      depthVisiting.delete(widgetId);
      depthCache.set(widgetId, depth);
      return depth;
    };

    candidates.sort((a, b) => {
      const depthDiff = hierarchyDepth(b.id) - hierarchyDepth(a.id);
      if (depthDiff !== 0) return depthDiff;
      const layoutA = resolvedWidgetLayout(a);
      const layoutB = resolvedWidgetLayout(b);
      const areaA = layoutA.w * layoutA.h;
      const areaB = layoutB.w * layoutB.h;
      if (areaA !== areaB) return areaA - areaB;
      return layoutB.y - layoutA.y || layoutB.x - layoutA.x || a.id.localeCompare(b.id);
    });

    return candidates[0] ?? null;
  }

  function areaOverlapsData(
    xStart: number,
    xEnd: number,
    yStart: number,
    yEnd: number,
    ignoreWidgetId?: string,
    parentLayoutId?: string
  ): boolean {
    return widgets.some((widget) => {
      if (widget.id === ignoreWidgetId) return false;
      if (isHiddenByCollapsedAccordion(widget)) return false;
      if (parentLayoutId !== undefined && widget.parentLayoutId !== parentLayoutId) return false;
      const layout = resolvedWidgetLayout(widget);
      const widgetYEnd = layout.y + layout.h;
      if (widgetYEnd <= yStart || layout.y >= yEnd) return false;
      const widgetXEnd = layout.x + layout.w;
      return !(widgetXEnd <= xStart || layout.x >= xEnd);
    });
  }

  function resolveLayoutDropPosition(
    layoutWidget: DashboardWidget,
    fallback: Pick<WidgetLayout, 'x' | 'y'>,
    dropSize: Pick<WidgetLayout, 'w' | 'h'> = DEFAULT_SIGNAL_DROP_SIZE,
    ignoreDataWidgetId?: string
  ): Pick<WidgetLayout, 'x' | 'y'> {
    const layoutBounds = resolveLayoutContentBounds(layoutWidget);
    const xStart = layoutBounds.x;
    const xEnd = layoutBounds.x + layoutBounds.w;
    const yStart = layoutBounds.y;
    const yEnd = layoutBounds.y + layoutBounds.h;
    const dropW = Math.max(1, Math.min(dropSize.w, xEnd - xStart));
    const dropH = Math.max(1, dropSize.h);
    const fallbackX = Math.max(xStart, Math.min(xEnd - dropW, fallback.x));
    const fallbackY = Math.max(yStart, Math.min(yEnd - dropH, fallback.y));

    if (layoutWidget.kind === 'layout_accordion') {
      return { x: xStart, y: yStart };
    }

    if (layoutWidget.kind === 'layout_list') {
      const listDropW = xEnd - xStart;
      const fallbackRow = Math.floor(fallbackY);
      const maxRow = yEnd - dropH;
      for (let row = fallbackRow; row <= maxRow; row++) {
        if (
          !areaOverlapsData(
            xStart,
            xStart + listDropW,
            row,
            row + dropH,
            ignoreDataWidgetId ?? layoutWidget.id,
            layoutWidget.id
          )
        ) {
          return { x: xStart, y: row };
        }
      }
      for (let row = Math.floor(yStart); row < fallbackRow; row++) {
        if (
          !areaOverlapsData(
            xStart,
            xStart + listDropW,
            row,
            row + dropH,
            ignoreDataWidgetId ?? layoutWidget.id,
            layoutWidget.id
          )
        ) {
          return { x: xStart, y: row };
        }
      }
      return { x: xStart, y: yStart };
    }

    if (layoutWidget.kind === 'layout_grid') {
      const spec = resolveGridLayoutSpec(layoutWidget);
      if (!spec) {
        return { x: xStart, y: yStart };
      }
      const gridBounds = spec.parent;
      const gridXStart = gridBounds.x;
      const gridYStart = gridBounds.y;
      const gridXEnd = gridBounds.x + gridBounds.w;
      const gridYEnd = gridBounds.y + gridBounds.h;
      const gridDropW = Math.max(1, Math.min(dropW, gridXEnd - gridXStart));
      const gridDropH = Math.max(1, Math.min(dropH, gridYEnd - gridYStart));
      const gridFallbackX = Math.max(gridXStart, Math.min(gridXEnd - gridDropW, fallback.x));
      const gridFallbackY = Math.max(gridYStart, Math.min(gridYEnd - gridDropH, fallback.y));
      const fallbackCellLayout = gridCellLayoutFromAbsolute(layoutWidget, {
        x: gridFallbackX,
        y: gridFallbackY,
        w: gridDropW,
        h: gridDropH
      });
      if (!fallbackCellLayout) {
        return { x: gridXStart, y: gridYStart };
      }

      const cellW = Math.max(1, Math.min(spec.columns, fallbackCellLayout.w));
      const cellH = Math.max(1, Math.min(spec.rows, fallbackCellLayout.h));
      const maxCellCol = Math.max(1, spec.columns - cellW + 1);
      const maxCellRow = Math.max(1, spec.rows - cellH + 1);
      const startCellCol = Math.max(1, Math.min(maxCellCol, fallbackCellLayout.x));
      const startCellRow = Math.max(1, Math.min(maxCellRow, fallbackCellLayout.y));

      for (let rowOffset = 0; rowOffset < maxCellRow; rowOffset++) {
        const row = 1 + ((startCellRow - 1 + rowOffset) % maxCellRow);
        for (let colOffset = 0; colOffset < maxCellCol; colOffset++) {
          const col = 1 + ((startCellCol - 1 + colOffset) % maxCellCol);
          const candidate = absoluteLayoutFromGridCell(layoutWidget, {
            x: col,
            y: row,
            w: cellW,
            h: cellH
          });
          if (!candidate) continue;
          const candidateXEnd = candidate.x + candidate.w;
          const candidateYEnd = candidate.y + candidate.h;
          if (
            !areaOverlapsData(
              candidate.x,
              candidateXEnd,
              candidate.y,
              candidateYEnd,
              ignoreDataWidgetId ?? layoutWidget.id,
              layoutWidget.id
            )
          ) {
            return { x: candidate.x, y: candidate.y };
          }
        }
      }

      const fallbackAbsolute = absoluteLayoutFromGridCell(layoutWidget, {
        x: startCellCol,
        y: startCellRow,
        w: cellW,
        h: cellH
      });
      if (!fallbackAbsolute) {
        return { x: gridXStart, y: gridYStart };
      }
      return { x: fallbackAbsolute.x, y: fallbackAbsolute.y };
    }

    if (
      layoutWidget.kind === 'layout_section' ||
      layoutWidget.kind === 'layout_divider' ||
      layoutWidget.kind === 'layout_title'
    ) {
      return { x: xStart, y: yStart + 1 };
    }

    return fallback;
  }

  function onSurfaceDragOver(event: DragEvent) {
    const transfer = event.dataTransfer;
    if (!transfer) return;
    const hasSignalId = transfer.types.includes(SIGNAL_DRAG_TYPE);
    const hasTopicPath = transfer.types.includes(TOPIC_DRAG_TYPE);
    const hasLayoutKind = transfer.types.includes(LAYOUT_DRAG_TYPE);
    const hasFallback = transfer.types.includes('text/plain');
    if (!hasSignalId && !hasTopicPath && !hasLayoutKind && !hasFallback) return;

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

    const rawLayout = transfer.getData(LAYOUT_DRAG_TYPE).trim();
    const fallback = transfer.getData('text/plain').trim();
    const topicRaw =
      transfer.getData(TOPIC_DRAG_TYPE).trim() ||
      (fallback.startsWith('topic:') ? fallback.slice('topic:'.length).trim() : '');
    const layoutKindRaw =
      rawLayout || (fallback.startsWith('layout:') ? fallback.slice('layout:'.length).trim() : '');
    const signalRaw = transfer.getData(SIGNAL_DRAG_TYPE).trim() || fallback;

    event.preventDefault();
    if (!gridEl) {
      if (isLayoutWidgetKind(layoutKindRaw)) {
        onDropLayoutTool(layoutKindRaw, { x: 1, y: 1 }, null);
        return;
      }
      if (topicRaw) {
        onDropTopic(topicRaw, { x: 1, y: 1 }, null);
        return;
      }
      const signalId = Number(signalRaw);
      if (!Number.isFinite(signalId) || signalId <= 0) return;
      onDropSignal(signalId, { x: 1, y: 1 }, null);
      return;
    }

    const rect = gridEl.getBoundingClientRect();
    const localX = event.clientX - rect.left + gridEl.scrollLeft;
    const localY = event.clientY - rect.top + gridEl.scrollTop;
    const step = cellSize + GRID_GAP;
    const x = Math.max(1, localX / step + 1);
    const y = Math.max(1, localY / step + 1);
    const fallbackPosition = { x, y };
    const canvasSnappedPosition = {
      x: Math.max(1, Math.round(x)),
      y: Math.max(1, Math.round(y))
    };
    const rawLayoutDropTarget = findLayoutWidgetAtCell(fallbackPosition);
    const layoutDropTarget =
      rawLayoutDropTarget?.kind === 'layout_accordion' &&
      accordionHasOtherChildren(rawLayoutDropTarget.id)
        ? null
        : rawLayoutDropTarget;
    const resolvedPosition = layoutDropTarget
      ? resolveLayoutDropPosition(layoutDropTarget, fallbackPosition, DEFAULT_SIGNAL_DROP_SIZE)
      : canvasSnappedPosition;
    const parentLayoutId =
      layoutDropTarget && isContainerLayoutKind(layoutDropTarget.kind) ? layoutDropTarget.id : null;

    if (isLayoutWidgetKind(layoutKindRaw)) {
      onDropLayoutTool(layoutKindRaw, resolvedPosition, parentLayoutId);
      return;
    }
    if (topicRaw) {
      onDropTopic(topicRaw, resolvedPosition, parentLayoutId);
      return;
    }

    const signalId = Number(signalRaw);
    if (!Number.isFinite(signalId) || signalId <= 0) return;
    onDropSignal(signalId, resolvedPosition, parentLayoutId);
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

    let moveRaf = 0;
    let pendingMove: { x: number; y: number } | null = null;
    const flushMove = () => {
      moveRaf = 0;
      if (!pendingMove) return;
      const { x, y } = pendingMove;
      pendingMove = null;
      applyDragAt(x, y);
    };

    const onMove = (event: PointerEvent) => {
      if (!drag) return;
      pendingMove = { x: event.clientX, y: event.clientY };
      if (moveRaf !== 0) return;
      moveRaf = window.requestAnimationFrame(flushMove);
    };
    const onUp = (event: PointerEvent) => {
      if (moveRaf !== 0) {
        window.cancelAnimationFrame(moveRaf);
        moveRaf = 0;
      }
      if (pendingMove) {
        const { x, y } = pendingMove;
        pendingMove = null;
        applyDragAt(x, y);
      }
      applyDragAt(event.clientX, event.clientY);
      endDrag();
    };
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target as HTMLElement | null;
      if (target?.closest('.widget-context-menu')) return;
      closeContextMenu();
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        closeContextMenu();
        return;
      }

      if (!contextMenu) return;

      const key = event.key.toLowerCase();
      const hasAccel = event.ctrlKey || event.metaKey;
      if (hasAccel && !event.altKey && key === 'c' && contextMenu.signalId !== null) {
        event.preventDefault();
        void copySignalValue(contextMenu.signalId).finally(() => {
          closeContextMenu();
        });
      }
    };
    const onDragEnd = () => {
      dragOverSurface = false;
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointerdown', onPointerDown);
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('dragend', onDragEnd);
    window.addEventListener('drop', onDragEnd);

    return () => {
      observer.disconnect();
      if (moveRaf !== 0) {
        window.cancelAnimationFrame(moveRaf);
      }
      if (copyToastTimer) {
        clearTimeout(copyToastTimer);
        copyToastTimer = null;
      }
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
      window.removeEventListener('pointerdown', onPointerDown);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('dragend', onDragEnd);
      window.removeEventListener('drop', onDragEnd);
    };
  });

  $effect(() => {
    onGridColumnsChange(gridColumns);
  });

  $effect(() => {
    if (!drag) return;
    const startLayout = clampLayout(drag.startLayout, gridColumns);
    const currentLayout = clampLayout(drag.currentLayout, gridColumns);
    if (layoutsEqual(startLayout, drag.startLayout) && layoutsEqual(currentLayout, drag.currentLayout)) {
      return;
    }
    drag = {
      ...drag,
      startLayout,
      currentLayout
    };
  });

  $effect(() => {
    widgets;
    if (typeof window === 'undefined' || typeof window.requestAnimationFrame !== 'function') {
      return;
    }
    const raf = window.requestAnimationFrame(() => {
      layoutMeasureTick += 1;
    });
    return () => {
      window.cancelAnimationFrame(raf);
    };
  });
</script>

<section class="dashboard-canvas panel">
  {#if widgets.length === 0}
    <div class="canvas-empty">
      <h3>Empty canvas</h3>
      <p>Use the floating Data Explorer to add your first panel.</p>
    </div>
  {/if}

  <div
    class="grid-surface"
    class:drag-over={dragOverSurface}
    class:dragging={Boolean(drag)}
    bind:this={gridEl}
    role="region"
    aria-label="Dashboard canvas grid"
    style={`--cell:${cellSize}px;--gap:${GRID_GAP}px;`}
    ondragover={onSurfaceDragOver}
    ondragleave={onSurfaceDragLeave}
    ondrop={onSurfaceDrop}
  >
    <div class="grid-spacer" style={`width:${canvasScrollWidth}px;height:${canvasScrollHeight}px;`} aria-hidden="true"></div>

    {#if widgets.length > 0}
      <div class="canvas-actions">
        <button class="btn" data-active={editMode} onclick={onToggleEditMode}>
          {editMode ? 'Edit mode' : 'View mode'}
        </button>
        <button class="btn" disabled={widgets.length === 0} onclick={onClearWidgets}>Clear</button>
      </div>
    {/if}

    {#each widgetsForRender as widget (widget.id)}
      {@const signal = resolvedSignalByWidgetId.get(widget.id) ?? null}
      {@const isLayoutWidget = isLayoutWidgetKind(widget.kind)}
      {@const accordionConfig = widget.kind === 'layout_accordion' ? readLayoutAccordionConfig(widget.config) : null}
      {@const accordionOpen = accordionConfig ? !accordionConfig.collapsed : false}
      {@const hiddenByAccordion = isHiddenByCollapsedAccordion(widget)}
      {#if !hiddenByAccordion}
        {@const layout = resolvedWidgetLayout(widget)}
        <div
          class={`widget-card ${isLayoutWidget ? 'layout-card' : ''} ${accordionOpen ? 'accordion-open' : ''} ${!isLayoutWidget && widget.parentLayoutId ? 'in-layout' : ''} ${selectedWidgetId === widget.id ? 'active' : ''} ${drag?.widgetId === widget.id ? 'dragging-card' : ''}`}
          data-widget-id={widget.id}
          style={widgetStyleFor(widget, layout)}
          role="button"
          tabindex="0"
          onclick={() => {
            closeContextMenu();
            onSelectWidget(widget.id, signal?.signal_id ?? null);
          }}
          oncontextmenu={(event) => openContextMenu(event, widget, signal?.signal_id ?? null)}
          onkeydown={(event) => {
            const key = event.key.toLowerCase();
            if (event.key === 'Enter' || event.key === ' ') {
              event.preventDefault();
              onSelectWidget(widget.id, signal?.signal_id ?? null);
              return;
            }
            if (event.key === 'Delete' || event.key === 'Backspace') {
              event.preventDefault();
              onRemoveWidget(widget.id);
              return;
            }
            if (key === 'i') {
              event.preventDefault();
              onRequestInspector();
            }
          }}
        >
          <button
            type="button"
            class="widget-head"
            onpointerdown={(event) => beginDrag(event, widget, 'move')}
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

          {#snippet widgetContent()}
            {#if isLayoutWidget}
              {#if widget.kind === 'layout_section'}
                <div class="layout-section-body">
                  <div class="layout-section-line"></div>
                </div>
              {:else if widget.kind === 'layout_divider'}
                <div class="layout-divider" aria-hidden="true"></div>
              {:else if widget.kind === 'layout_list'}
                <div class="layout-list" aria-hidden="true">
                  {#each Array.from({ length: Math.max(1, Math.min(10, layout.h)) }) as _, index (`list-${widget.id}-${index}`)}
                    <span></span>
                  {/each}
                </div>
              {:else if widget.kind === 'layout_accordion' && accordionConfig}
                <div class={`layout-accordion ${accordionConfig.collapsed ? 'collapsed' : ''}`}>
                  <button
                    type="button"
                    class="layout-accordion-head layout-accordion-toggle"
                    aria-label={`${accordionConfig.collapsed ? 'Expand' : 'Collapse'} ${widget.title}`}
                    aria-expanded={!accordionConfig.collapsed}
                    onclick={() => toggleAccordionCollapsed(widget)}
                  >
                    <strong>{widget.title}</strong>
                    <span class="layout-accordion-chevron">{accordionConfig.collapsed ? '>' : 'v'}</span>
                  </button>
                  <div class="layout-accordion-body">
                    {#if accordionConfig.collapsed}
                      <button
                        type="button"
                        class="layout-accordion-collapsed-fill layout-accordion-toggle"
                        aria-label={`Expand ${widget.title}`}
                        aria-expanded="false"
                        onclick={() => toggleAccordionCollapsed(widget)}
                      >
                        <span class="layout-accordion-collapsed-label">{widget.title}</span>
                        <span class="layout-accordion-chevron">></span>
                      </button>
                    {:else}
                      <div class="layout-list">
                        {#each Array.from({ length: Math.max(1, Math.min(8, layout.h)) }) as _, index (`accordion-${widget.id}-${index}`)}
                          <span></span>
                        {/each}
                      </div>
                    {/if}
                  </div>
                </div>
              {:else if widget.kind === 'layout_grid'}
                {@const layoutSpec = resolveGridLayoutSpec(widget)}
                {@const layoutGridConfig = readLayoutGridConfig(
                  widget.config,
                  Math.max(1, Math.floor(layout.w)),
                  Math.max(1, Math.floor(layout.h))
                )}
                {@const effectiveCols = layoutSpec?.columns ?? layoutGridConfig.columns}
                {@const effectiveRows = layoutSpec?.rows ?? layoutGridConfig.rows}
                <div
                  class="layout-grid"
                  style={`--layout-cols:${effectiveCols};--layout-rows:${effectiveRows};`}
                  aria-hidden="true"
                ></div>
              {:else if widget.kind === 'layout_spacer'}
                <div class="layout-spacer">Spacer</div>
              {:else if widget.kind === 'layout_title'}
                {@const titleConfig = readLayoutTitleConfig(widget.config, widget.title)}
                <div class="layout-title" style={`color:${titleConfig.color};`}>{titleConfig.text}</div>
              {/if}
            {:else if signal}
              {#if widget.kind === 'metric'}
                <div class="metric-value">{signal.value}</div>
              {:else if widget.kind === 'state'}
                <span class={`state-pill ${signal.value === 'true' ? 'on' : ''}`}>{signal.value}</span>
              {:else if widget.kind === 'trend'}
                <svg viewBox="0 0 100 34" preserveAspectRatio="none" aria-label="trend chart">
                  <path d={sparklinePath(historyBySignal.get(signal.signal_id) ?? [])} />
                </svg>
                <div class="meta-line">Current: {signal.value}</div>
              {:else if widget.kind === 'graph'}
                <GraphWidget
                  {signal}
                  {signalById}
                  {historyBySignal}
                  {historyTimeBySignal}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'controller'}
                <ControllerEditorWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                  onSendAction={onSendAction}
                />
              {:else if widget.kind === 'state_machine'}
                <StateMachineWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                  onSendAction={onSendAction}
                />
              {:else if widget.kind === 'motor'}
                <MotorWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'dio'}
                <DioWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'encoder'}
                <EncoderWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'imu'}
                <ImuWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'bar'}
                <BarMeterWidget
                  {signal}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'dial'}
                <DialWidget
                  {signal}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'compass'}
                <CompassWidget
                  {signal}
                />
              {:else if widget.kind === 'timer'}
                <TimerWidget
                  {signal}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'status_matrix'}
                <StatusMatrixWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'camera_overlay'}
                <CameraOverlayWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'dropdown'}
                <DropdownWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'toggle'}
                <ToggleWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'button_group'}
                <ButtonGroupWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'radio'}
                <RadioWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'input'}
                <InputWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'textarea'}
                <TextAreaWidget
                  {signal}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'field'}
                <FieldViewerWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'imu_3d'}
                <Imu3dWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'mech2d'}
                <Mech2dWidget
                  {signal}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'swerve_module'}
                <SwerveModuleWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                  onSendSet={onSendSet}
                />
              {:else if widget.kind === 'swerve_drive'}
                <SwerveDriveWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'differential_drive'}
                <DifferentialDriveWidget
                  {signal}
                  {signals}
                  {signalById}
                  configRaw={widget.config}
                />
              {:else if widget.kind === 'action'}
                <button
                  class="btn btn-danger"
                  disabled={!isActionSignal(signal)}
                  onclick={() => onSendAction(signal.signal_id)}
                >
                  Trigger action
                </button>
              {:else if widget.kind === 'tunable'}
                <div class="inline-set-row">
                  <input
                    value={widgetInputs[widget.id] ?? signal.value}
                    oninput={(e) => onWidgetInput(widget.id, (e.currentTarget as HTMLInputElement).value)}
                  />
                  <button
                    class="btn btn-primary"
                    onclick={() => onSendSet(signal.signal_id, widgetInputs[widget.id] ?? '')}
                  >
                    Set value
                  </button>
                </div>
              {:else}
                <pre>{signal.value}</pre>
              {/if}
            {:else}
              <div class="signal-unavailable">
                <strong>Signal unavailable</strong>
                <span class="signal-unavailable-hint">{unavailableSignalHint(widget)}</span>
                <small>Reconnect ARCP or remap this widget in Inspector.</small>
              </div>
            {/if}
          {/snippet}

          <div class="widget-body">
            {@render widgetContent()}
          </div>

          {#if editMode}
            <button
              class="resize-handle"
              title="Resize panel"
              aria-label="Resize panel"
              onpointerdown={(event) => beginDrag(event, widget, 'resize')}
              onclick={(event) => event.stopPropagation()}
            >
              <FontAwesomeIcon icon={faUpRightAndDownLeftFromCenter} class="resize-handle-icon" />
            </button>
          {/if}
        </div>
      {/if}
    {/each}
  </div>

  {#if contextMenu}
    <div class="widget-context-menu" style={menuStyle(contextMenu)} role="menu" aria-label="Widget actions">
      <button
        class="menu-item"
        role="menuitem"
        onclick={openInspector}
      >
        Open inspector
        <span>I</span>
      </button>
      {#if contextMenu.signalId !== null}
        <button
          class="menu-item"
          role="menuitem"
          onclick={async (event) => {
            event.preventDefault();
            event.stopPropagation();
            await copySignalValue(contextMenu.signalId);
            closeContextMenu();
          }}
        >
          Copy value
          <span>Ctrl+C</span>
        </button>
      {/if}
      <button
        class="menu-item danger"
        role="menuitem"
        onclick={(event) => {
          event.preventDefault();
          event.stopPropagation();
          onRemoveWidget(contextMenu.widgetId);
          closeContextMenu();
        }}
      >
        Delete panel
        <span>Del</span>
      </button>
    </div>
  {/if}

  {#if copyToastVisible}
    <div class="copy-toast" role="status" aria-live="polite">
      <FontAwesomeIcon icon={faClipboard} class="copy-toast-icon" />
      <span>Copied</span>
    </div>
  {/if}
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

  .widget-card {
    position: absolute;
    left: 0;
    top: 0;
    box-sizing: border-box;
    border: 1px solid rgba(99, 115, 140, 0.5);
    border-radius: 7px;
    background: rgba(26, 33, 45, 0.98);
    padding: 0.12rem 0.28rem 0.24rem;
    display: grid;
    grid-template-rows: minmax(0.82rem, calc(var(--cell) * 0.46)) minmax(0, 1fr);
    gap: 0;
    cursor: pointer;
    overflow: hidden;
    box-shadow: 0 4px 12px rgba(0, 0, 0, 0.22);
    content-visibility: auto;
    contain-intrinsic-size: 220px 140px;
    contain: layout paint;
  }

  .widget-card.dragging-card {
    will-change: transform;
  }

  .grid-surface.dragging .widget-card {
    box-shadow: none;
  }

  .widget-card.layout-card {
    border-style: dashed;
    background: rgba(30, 37, 49, 0.86);
    padding: 0.06rem 0.14rem 0.12rem;
    grid-template-rows: minmax(0.46rem, calc(var(--cell) * 0.2)) minmax(0, 1fr);
  }

  .widget-card.layout-card.accordion-open {
    overflow: visible;
  }

  .widget-card.in-layout {
    border-color: rgba(56, 189, 248, 0.55);
    background: rgba(21, 32, 44, 0.98);
    box-shadow: inset 0 0 0 1px rgba(56, 189, 248, 0.2), 0 4px 10px rgba(0, 0, 0, 0.25);
  }

  .widget-card.active {
    border-color: rgba(180, 35, 45, 0.9);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.34), 0 6px 14px rgba(0, 0, 0, 0.28);
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

  .widget-card.layout-card.accordion-open .layout-accordion {
    box-shadow: 0 8px 20px rgba(0, 0, 0, 0.34);
  }

  .layout-section-body {
    width: 100%;
    height: 100%;
    display: flex;
    align-items: center;
  }

  .layout-section-line {
    width: 100%;
    border-top: 1px dashed rgba(180, 35, 45, 0.45);
  }

  .layout-divider {
    width: 100%;
    border-top: 1px solid var(--border-emphasis);
  }

  .layout-list {
    width: 100%;
    height: 100%;
    display: flex;
    flex-direction: column;
    gap: 0.16rem;
    justify-content: flex-start;
  }

  .layout-list span {
    display: block;
    width: 100%;
    flex: 0 0 0.42rem;
    min-height: 0.42rem;
    border-radius: 2px;
    background: rgba(153, 164, 180, 0.2);
    border: 1px solid rgba(153, 164, 180, 0.2);
  }

  .layout-accordion {
    width: 100%;
    height: 100%;
    min-height: 1.2rem;
    border: 1px solid rgba(153, 164, 180, 0.24);
    border-radius: 5px;
    background: rgba(28, 35, 47, 0.72);
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    overflow: hidden;
  }

  .layout-accordion-head {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.3rem;
    padding: 0.18rem 0.28rem;
    border-bottom: 1px solid rgba(153, 164, 180, 0.24);
    background: rgba(40, 49, 66, 0.8);
  }

  .layout-accordion-toggle {
    width: 100%;
    border: 0;
    margin: 0;
    text-align: left;
    font: inherit;
    color: inherit;
    cursor: pointer;
  }

  .layout-accordion-toggle:focus-visible {
    outline: 2px solid rgba(138, 183, 255, 0.85);
    outline-offset: -2px;
  }

  .layout-accordion-head strong {
    min-width: 0;
    margin: 0;
    font-size: 0.66rem;
    font-weight: 700;
    line-height: 1.1;
    color: var(--text-strong);
    text-overflow: ellipsis;
    white-space: nowrap;
    overflow: hidden;
  }

  .layout-accordion-chevron {
    font-size: 0.62rem;
    color: var(--text-soft);
    line-height: 1;
  }

  .layout-accordion-body {
    min-height: 0;
    padding: 0.14rem;
    overflow: hidden;
  }

  .layout-accordion.collapsed .layout-accordion-body {
    display: flex;
    align-items: stretch;
    justify-content: stretch;
    padding: 0.08rem;
  }

  .layout-accordion.collapsed {
    grid-template-rows: minmax(0, 1fr);
  }

  .layout-accordion.collapsed .layout-accordion-head {
    display: none;
  }

  .layout-accordion-collapsed-fill {
    flex: 1 1 auto;
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.18rem;
    padding: 0.1rem 0.2rem;
    width: 100%;
    height: 100%;
    min-height: 0.36rem;
    border: 1px solid rgba(153, 164, 180, 0.28);
    border-radius: 4px;
    color: var(--text-soft);
    background:
      linear-gradient(180deg, rgba(80, 96, 120, 0.2), rgba(45, 57, 76, 0.3)),
      repeating-linear-gradient(
        90deg,
        rgba(153, 164, 180, 0.14),
        rgba(153, 164, 180, 0.14) 5px,
        transparent 5px,
        transparent 10px
      );
  }

  .layout-accordion-collapsed-fill:focus-visible {
    outline: 2px solid rgba(138, 183, 255, 0.85);
    outline-offset: -2px;
  }

  .layout-accordion-collapsed-label {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: 0.58rem;
    font-weight: 700;
    color: var(--text-strong);
  }

  .layout-grid {
    width: 100%;
    height: 100%;
    min-height: 1.4rem;
    border: 1px solid rgba(153, 164, 180, 0.2);
    border-radius: 4px;
    background:
      linear-gradient(to right, rgba(153, 164, 180, 0.24) 1px, transparent 1px),
      linear-gradient(to bottom, rgba(153, 164, 180, 0.2) 1px, transparent 1px);
    background-size:
      calc(100% / max(1, var(--layout-cols, 4))) 100%,
      100% calc(100% / max(1, var(--layout-rows, 4)));
    background-position: 0 0, 0 0;
  }

  .layout-spacer {
    width: 100%;
    text-align: center;
    color: var(--text-soft);
    font-size: 0.66rem;
    border: 1px dashed rgba(153, 164, 180, 0.35);
    border-radius: 5px;
    padding: 0.25rem;
  }

  .layout-title {
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    text-align: center;
    font-family: var(--font-display);
    font-size: clamp(1rem, calc(var(--cell) * 0.6), 2.2rem);
    line-height: 1.05;
    font-weight: 800;
    letter-spacing: 0.01em;
    overflow-wrap: anywhere;
    padding: 0.1rem;
  }

  .inline-set-row {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    width: 100%;
    min-width: 0;
  }

  .inline-set-row input {
    flex: 1 1 auto;
    min-width: 0;
  }

  .inline-set-row .btn {
    flex: 0 0 auto;
    white-space: nowrap;
  }

  .metric-value {
    font-size: 1.08rem;
    font-family: var(--font-display);
    line-height: 1.08;
    white-space: normal;
    overflow-wrap: anywhere;
    color: var(--text-strong);
  }

  .meta-line {
    color: var(--text-soft);
    font-size: 0.68rem;
    line-height: 1.2;
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .signal-unavailable {
    display: grid;
    gap: 0.18rem;
    border: 1px dashed rgba(153, 164, 180, 0.4);
    border-radius: 6px;
    padding: 0.34rem 0.42rem;
    background:
      linear-gradient(180deg, rgba(69, 79, 97, 0.3), rgba(36, 43, 56, 0.24)),
      repeating-linear-gradient(
        135deg,
        rgba(153, 164, 180, 0.08),
        rgba(153, 164, 180, 0.08) 6px,
        transparent 6px,
        transparent 12px
      );
  }

  .signal-unavailable strong {
    font-size: 0.7rem;
    line-height: 1.15;
    color: var(--text-strong);
  }

  .signal-unavailable-hint {
    font-family: var(--font-mono);
    font-size: 0.64rem;
    color: var(--text-soft);
    overflow-wrap: anywhere;
  }

  .signal-unavailable small {
    font-size: 0.62rem;
    line-height: 1.2;
    color: color-mix(in srgb, var(--text-soft) 86%, white 14%);
  }

  .state-pill {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    border-radius: 999px;
    border: 1px solid var(--border-subtle);
    min-width: 74px;
    padding: 0.18rem 0.45rem;
    font-family: var(--font-display);
    background: var(--surface-3);
    color: var(--text);
    font-size: 0.72rem;
  }

  .state-pill.on {
    background: rgba(180, 35, 45, 0.24);
    color: #ffd7dd;
    border-color: rgba(180, 35, 45, 0.48);
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 0;
    border-radius: 6px;
    background: #20293a;
    border: 1px solid var(--border-subtle);
  }

  path {
    fill: none;
    stroke: var(--brand);
    stroke-width: 1.6;
  }

  pre {
    margin: 0;
    white-space: pre-wrap;
    word-break: break-word;
    font-family: var(--font-mono);
    font-size: 0.7rem;
  }

  .resize-handle {
    position: absolute;
    right: 0.36rem;
    bottom: 0.26rem;
    width: 1.1rem;
    height: 1.1rem;
    display: inline-grid;
    place-items: center;
    border: none;
    background: transparent;
    color: var(--text-soft);
    cursor: nwse-resize;
    padding: 0;
    user-select: none;
    touch-action: none;
    z-index: 6;
  }

  .resize-handle-icon {
    width: 0.54rem;
    height: 0.54rem;
    display: block;
  }

  .widget-context-menu {
    position: fixed;
    z-index: 2000;
    width: 210px;
    border: 1px solid var(--border-subtle);
    border-radius: 9px;
    background: #1b2230;
    box-shadow: 0 14px 32px rgba(0, 0, 0, 0.45);
    padding: 0.2rem;
    display: grid;
    gap: 0.1rem;
  }

  .menu-item {
    border: 1px solid transparent;
    border-radius: 6px;
    background: transparent;
    color: var(--text);
    padding: 0.43rem 0.52rem;
    font-size: 0.74rem;
    text-align: left;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .menu-item span {
    color: var(--text-soft);
    font-family: var(--font-mono);
    font-size: 0.68rem;
  }

  .menu-item:hover {
    border-color: var(--border-subtle);
    background: #273146;
  }

  .menu-item.danger {
    color: #fecaca;
  }

  .menu-item.danger:hover {
    border-color: rgba(248, 113, 113, 0.45);
    background: rgba(127, 29, 29, 0.36);
  }

  .copy-toast {
    position: absolute;
    left: 50%;
    bottom: 0.78rem;
    transform: translateX(-50%);
    z-index: 2101;
    pointer-events: none;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.46rem;
    min-width: 9.6rem;
    border: 1px solid rgba(239, 68, 68, 0.75);
    border-radius: 12px;
    background: linear-gradient(180deg, rgba(185, 28, 28, 0.97), rgba(127, 29, 29, 0.97));
    color: #fff1f2;
    padding: 0.52rem 0.96rem;
    font-size: 0.92rem;
    font-weight: 600;
    line-height: 1;
    box-shadow:
      0 10px 24px rgba(0, 0, 0, 0.4),
      inset 0 1px 0 rgba(255, 255, 255, 0.12);
  }

  .copy-toast-icon {
    width: 0.88rem;
    height: 0.88rem;
    display: block;
    color: #ffe4e6;
  }
</style>
