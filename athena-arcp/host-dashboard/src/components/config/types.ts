import type { SignalRow } from '../../lib/arcp';
import type { DashboardWidget, WidgetConfigRecord } from '../../lib/dashboard';
import type { EditorPools } from '../../lib/widget-config/editor-pools';

/** Request to pick a signal in the explorer; the editor supplies the candidates and what to do with the pick. */
export type SignalMapRequest = {
  title: string;
  candidates: SignalRow[];
  selectedSignalId: number | null;
  allowNone: boolean;
  noneLabel: string;
  onPick: (signalId: number | null) => void;
};

export type ConfigSetter = (config: WidgetConfigRecord | undefined) => void;

/** Props for sections that edit a widget bound to a signal (the signal is resolved by the editor). */
export type SignalSectionProps = {
  selectedWidget: DashboardWidget;
  primarySignal: SignalRow;
  selectedSignal: SignalRow | null;
  signals: SignalRow[];
  signalById: Map<number, SignalRow>;
  pools: EditorPools;
  setConfig: ConfigSetter;
};

/** Props for layout blocks, which have no signal. */
export type LayoutSectionProps = {
  selectedWidget: DashboardWidget;
  setConfig: ConfigSetter;
};
