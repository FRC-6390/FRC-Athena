/** Right-click menu on a classic widget. The model says what to show; the component renders it. */
export type WidgetMenuState = {
  widgetId: string;
  signalId: number | null;
  x: number;
  y: number;
};

export type WidgetMenuAction = 'inspector' | 'copy' | 'delete';

export type WidgetMenuItem = {
  action: WidgetMenuAction;
  label: string;
  shortcut: string;
  danger: boolean;
};

/** Items for a widget menu. Copy is offered only when the widget shows a signal. */
export function widgetMenuItems(menu: Pick<WidgetMenuState, 'signalId'>): WidgetMenuItem[] {
  const items: WidgetMenuItem[] = [{ action: 'inspector', label: 'Open inspector', shortcut: 'I', danger: false }];
  if (menu.signalId !== null) {
    items.push({ action: 'copy', label: 'Copy value', shortcut: 'Ctrl+C', danger: false });
  }
  items.push({ action: 'delete', label: 'Delete panel', shortcut: 'Del', danger: true });
  return items;
}

/** Keeps the menu on screen: at least 8px from the top-left and clear of the viewport's bottom-right. */
export function widgetMenuPosition(
  menu: Pick<WidgetMenuState, 'x' | 'y'>,
  viewport: { width: number; height: number }
): { left: number; top: number } {
  const x = Math.min(menu.x, viewport.width - 220);
  const y = Math.min(menu.y, viewport.height - 120);
  return { left: Math.max(8, x), top: Math.max(8, y) };
}

/** Inline `left/top` style for the menu. */
export function widgetMenuStyle(menu: Pick<WidgetMenuState, 'x' | 'y'>, viewport: { width: number; height: number }): string {
  const { left, top } = widgetMenuPosition(menu, viewport);
  return `left:${left}px;top:${top}px;`;
}

/** Ctrl/Cmd+C with an open menu for a signal-bound widget copies that signal's value. */
export function isCopyShortcut(
  event: { key: string; ctrlKey: boolean; metaKey: boolean; altKey: boolean },
  menu: Pick<WidgetMenuState, 'signalId'> | null
): boolean {
  if (!menu || menu.signalId === null) return false;
  const hasAccel = event.ctrlKey || event.metaKey;
  return hasAccel && !event.altKey && event.key.toLowerCase() === 'c';
}
