/** Keyboard actions on a focused classic widget card. */
export type WidgetKeyAction = 'select' | 'remove' | 'inspector';

export function widgetKeyAction(key: string): WidgetKeyAction | null {
  if (key === 'Enter' || key === ' ') return 'select';
  if (key === 'Delete' || key === 'Backspace') return 'remove';
  if (key.toLowerCase() === 'i') return 'inspector';
  return null;
}
