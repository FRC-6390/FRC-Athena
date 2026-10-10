import { allWidgets } from '../../lib/board/registry';
import { registerCoreWidgets } from './core';
import { registerMatchWidgets } from './match';

/**
 * Registers every board widget once. Widget packs add their own `register…Widgets()` here.
 */
let registered = false;

export function registerAllWidgets(): void {
  if (registered || allWidgets().length > 0) {
    registered = true;
    return;
  }
  registered = true;
  registerCoreWidgets();
  registerMatchWidgets();
}
