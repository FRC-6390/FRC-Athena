import type { SignalRow } from '../arcp';
import type { WidgetConfigRecord } from '../dashboard';

/**
 * Props every classic signal widget receives from the canvas. Each widget component declares the subset it
 * uses, so a widget never needs a prop it does not read.
 */
export type ClassicWidgetProps = {
  signal: SignalRow;
  signals: SignalRow[];
  signalById: Map<number, SignalRow>;
  widgetId: string;
  configRaw: WidgetConfigRecord | undefined;
  historyBySignal: Map<number, number[]>;
  historyTimeBySignal: Map<number, number[]>;
  nowMs: number;
  /** Text typed into a tunable widget that has not been sent yet. */
  inputValue: string | undefined;
  onInput: (value: string) => void;
  onSendSet: (signalId: number, valueRaw: string) => void;
  onSendAction: (signalId: number) => void;
};
