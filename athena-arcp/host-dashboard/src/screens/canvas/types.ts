import type { SignalRow } from '../../lib/arcp';
import type { DashboardLayoutState, WidgetConfigRecord, WidgetKind } from '../../lib/dashboard';

/** Request from a widget inspector to pick a signal in the explorer. The explorer calls `onPick` once. */
export type SignalMapRequest = {
  title: string;
  candidates: SignalRow[];
  selectedSignalId: number | null;
  allowNone: boolean;
  noneLabel: string;
  onPick: (signalId: number | null) => void;
};

/** Imperative API the host uses from outside the canvas (bound with `bind:this`). */
export type CanvasDashboardsApi = {
  undo(): void;
  redo(): void;
  copySelectedWidget(): boolean;
  pasteCopiedWidget(): boolean;
  importLayoutFromFile(): Promise<void>;
  exportLayoutToFile(): Promise<void>;
  commitLayout(next: DashboardLayoutState, options?: { recordHistory?: boolean; clearRedo?: boolean }): void;
  createWidgetFromSignal(signal: SignalRow, kind: WidgetKind): void;
  showAsWidgetKind(kind: WidgetKind): void;
  removeWidget(widgetId: string): void;
  updateWidgetTitle(widgetId: string, title: string): void;
  updateWidgetConfig(widgetId: string, config: WidgetConfigRecord | undefined): void;
  requestSignalMap(request: SignalMapRequest): void;
  setGridColumns(columns: number): void;
};
