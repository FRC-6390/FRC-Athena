import type { SignalRow } from '../../lib/arcp';
import type { DashboardWidget, WidgetLayout } from '../../lib/dashboard';
import type { DragMode } from '../../lib/canvas/model';
import type { WidgetKeyAction } from '../../lib/canvas/selection';

/** Data every classic widget body may read from the canvas. Rebuilt whenever the signal table changes. */
export type CanvasRuntime = {
  signals: SignalRow[];
  signalById: Map<number, SignalRow>;
  historyBySignal: Map<number, number[]>;
  historyTimeBySignal: Map<number, number[]>;
  nowMs: number;
  widgetInputs: Record<string, string>;
  onWidgetInput: (widgetId: string, value: string) => void;
  onSendSet: (signalId: number, valueRaw: string) => void;
  onSendAction: (signalId: number) => void;
};

/** One card on the canvas, fully resolved for drawing. */
export type CanvasFrame = {
  widget: DashboardWidget;
  /** Drawn layout in grid units (live drag applied). */
  layout: WidgetLayout;
  /** Inline position/size/stacking style. */
  style: string;
  isLayout: boolean;
  /** Signal bound to a data widget; null for layout blocks and unresolved widgets. */
  signal: SignalRow | null;
  /** Human hint shown when `signal` is null. */
  unavailableHint: string;
  /** Accordion state, for accordion blocks only. */
  accordionCollapsed: boolean;
  /** Column/row counts for grid blocks. */
  gridCounts: { columns: number; rows: number } | null;
  /** Card sits inside a layout container. */
  inLayout: boolean;
  /** Card is inside an open accordion and needs the "open" styling. */
  accordionOpen: boolean;
};

export type FrameHandlers = {
  select: (widget: DashboardWidget) => void;
  contextMenu: (event: MouseEvent, widget: DashboardWidget) => void;
  key: (action: WidgetKeyAction, widget: DashboardWidget) => void;
  beginDrag: (event: PointerEvent, widget: DashboardWidget, mode: DragMode) => void;
  toggleAccordion: (widget: DashboardWidget) => void;
};
