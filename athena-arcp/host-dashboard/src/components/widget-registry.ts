import type { WidgetComponent, WidgetModuleId } from '../lib/widgets/contract';
import PlotWidget from './widgets/PlotWidget.svelte';

/**
 * Svelte components for new-contract widget modules. Legacy kinds are still drawn by WidgetCanvas and
 * have no entry here.
 */
export const WIDGET_COMPONENTS: Partial<Record<WidgetModuleId, WidgetComponent<never>>> = {
  plot: PlotWidget as unknown as WidgetComponent<never>
};

export function componentFor(id: WidgetModuleId): WidgetComponent<never> | undefined {
  return WIDGET_COMPONENTS[id];
}
