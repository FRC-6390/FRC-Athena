import { isLayoutWidgetKind, type DashboardWidget, type WidgetLayout } from '../dashboard';
import type { DragState } from './drag';
import { boxStyle, pixelRectFor } from './grid-math';
import { type CanvasContext } from './model';
import { accordionStackLayer, isDescendantOf } from './tree';

/**
 * Inline style for one card: its static box, the live box while a move drag is in progress, or a box
 * lifted above its neighbours (accordion stacking; contents of a moving container travel with it).
 */
export function frameStyleFor(input: {
  ctx: CanvasContext;
  drag: DragState | null;
  cellSize: number;
  widget: DashboardWidget;
  layout: WidgetLayout;
}): string {
  const { ctx, drag, cellSize, widget, layout } = input;
  const withoutTrailingGap = Boolean(widget.parentLayoutId);
  const stackLayer = accordionStackLayer(ctx.byId, widget);
  const stackedZIndex = stackLayer > 0 ? 10 + stackLayer : undefined;
  const staticBox = pixelRectFor(layout, cellSize, withoutTrailingGap);
  if (!drag || drag.mode !== 'move') {
    return boxStyle(staticBox, stackedZIndex);
  }

  const pointerDx = drag.pointerX - drag.startX;
  const pointerDy = drag.pointerY - drag.startY;
  const movingWidget = ctx.byId.get(drag.widgetId) ?? null;

  if (widget.id !== drag.widgetId) {
    if (movingWidget && isLayoutWidgetKind(movingWidget.kind) && isDescendantOf(ctx.byId, widget, movingWidget.id)) {
      const box = pixelRectFor(layout, cellSize, true);
      const left = box.left + pointerDx;
      const top = box.top + pointerDy;
      return `transform:translate3d(${left}px,${top}px,0);width:${Math.max(1, box.width)}px;height:${Math.max(1, box.height)}px;z-index:22;`;
    }
    return boxStyle(staticBox, stackedZIndex);
  }

  const startBox = pixelRectFor(drag.startLayout, cellSize, withoutTrailingGap);
  const left = startBox.left + pointerDx;
  const top = startBox.top + pointerDy;
  const draggingLayoutShell = isLayoutWidgetKind(widget.kind);
  const zIndex = draggingLayoutShell ? 16 : 20;
  const opacity = draggingLayoutShell ? 'opacity:0.88;' : '';
  return `transform:translate3d(${left}px,${top}px,0);width:${Math.max(1, staticBox.width)}px;height:${Math.max(1, staticBox.height)}px;z-index:${zIndex};${opacity}`;
}
