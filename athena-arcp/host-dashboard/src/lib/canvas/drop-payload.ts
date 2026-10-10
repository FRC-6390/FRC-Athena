import { isLayoutWidgetKind, type LayoutToolKind } from '../dashboard';

/** Drag data types the canvas accepts. Signals and topics also accept plain text (`topic:` / `layout:` prefixes). */
export const SIGNAL_DRAG_TYPE = 'application/x-arcp-signal-id';
export const TOPIC_DRAG_TYPE = 'application/x-arcp-topic-path';
export const LAYOUT_DRAG_TYPE = 'application/x-arcp-layout-kind';

export type DropPayload =
  | { kind: 'layout'; layoutKind: LayoutToolKind }
  | { kind: 'topic'; topicPath: string }
  | { kind: 'signal'; signalId: number }
  | null;

/** True when the drag carries something the canvas can place. */
export function acceptsDragTypes(types: readonly string[]): boolean {
  return (
    types.includes(SIGNAL_DRAG_TYPE) ||
    types.includes(TOPIC_DRAG_TYPE) ||
    types.includes(LAYOUT_DRAG_TYPE) ||
    types.includes('text/plain')
  );
}

/**
 * Reads what was dropped. Typed payloads win; the plain-text fallback carries `layout:`, `topic:` or a
 * bare signal id.
 */
export function parseDropPayload(getData: (type: string) => string): DropPayload {
  const rawLayout = getData(LAYOUT_DRAG_TYPE).trim();
  const fallback = getData('text/plain').trim();
  const topicRaw =
    getData(TOPIC_DRAG_TYPE).trim() || (fallback.startsWith('topic:') ? fallback.slice('topic:'.length).trim() : '');
  const layoutKindRaw =
    rawLayout || (fallback.startsWith('layout:') ? fallback.slice('layout:'.length).trim() : '');
  const signalRaw = getData(SIGNAL_DRAG_TYPE).trim() || fallback;

  if (isLayoutWidgetKind(layoutKindRaw)) {
    return { kind: 'layout', layoutKind: layoutKindRaw as LayoutToolKind };
  }
  if (topicRaw) return { kind: 'topic', topicPath: topicRaw };
  const signalId = Number(signalRaw);
  if (!Number.isFinite(signalId) || signalId <= 0) return null;
  return { kind: 'signal', signalId };
}
