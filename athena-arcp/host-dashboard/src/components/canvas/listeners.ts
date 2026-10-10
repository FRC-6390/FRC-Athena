import type { WidgetLayout } from '../../lib/dashboard';
import { isCopyShortcut, type WidgetMenuState } from '../../lib/canvas/context-menu';

/**
 * Window-level pointer and keyboard wiring for the canvas. Pointer moves are coalesced to one drag update per
 * animation frame; releasing the pointer always applies the last sample before ending the drag.
 */
export function installCanvasListeners(callbacks: {
  isDragging: () => boolean;
  applyDragAt: (x: number, y: number) => void;
  endDrag: () => void;
  closeMenu: () => void;
  menu: () => WidgetMenuState | null;
  copySignal: (signalId: number) => void;
  dragEnded: () => void;
}): () => void {
  let moveRaf = 0;
  let pendingMove: Pick<WidgetLayout, 'x' | 'y'> | null = null;
  const flushMove = () => {
    moveRaf = 0;
    if (!pendingMove) return;
    const { x, y } = pendingMove;
    pendingMove = null;
    callbacks.applyDragAt(x, y);
  };

  const onMove = (event: PointerEvent) => {
    if (!callbacks.isDragging()) return;
    pendingMove = { x: event.clientX, y: event.clientY };
    if (moveRaf !== 0) return;
    moveRaf = window.requestAnimationFrame(flushMove);
  };
  const onUp = (event: PointerEvent) => {
    if (moveRaf !== 0) {
      window.cancelAnimationFrame(moveRaf);
      moveRaf = 0;
    }
    if (pendingMove) {
      const { x, y } = pendingMove;
      pendingMove = null;
      callbacks.applyDragAt(x, y);
    }
    callbacks.applyDragAt(event.clientX, event.clientY);
    callbacks.endDrag();
  };
  const onPointerDown = (event: PointerEvent) => {
    const target = event.target as HTMLElement | null;
    if (target?.closest('.widget-context-menu')) return;
    callbacks.closeMenu();
  };
  const onKeyDown = (event: KeyboardEvent) => {
    if (event.key === 'Escape') {
      callbacks.closeMenu();
      return;
    }
    const menu = callbacks.menu();
    if (isCopyShortcut(event, menu) && menu?.signalId != null) {
      event.preventDefault();
      callbacks.copySignal(menu.signalId);
    }
  };

  window.addEventListener('pointermove', onMove);
  window.addEventListener('pointerup', onUp);
  window.addEventListener('pointerdown', onPointerDown);
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('dragend', callbacks.dragEnded);
  window.addEventListener('drop', callbacks.dragEnded);

  return () => {
    if (moveRaf !== 0) window.cancelAnimationFrame(moveRaf);
    window.removeEventListener('pointermove', onMove);
    window.removeEventListener('pointerup', onUp);
    window.removeEventListener('pointerdown', onPointerDown);
    window.removeEventListener('keydown', onKeyDown);
    window.removeEventListener('dragend', callbacks.dragEnded);
    window.removeEventListener('drop', callbacks.dragEnded);
  };
}
