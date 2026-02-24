<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faUpRightAndDownLeftFromCenter, faXmark } from '@fortawesome/free-solid-svg-icons';
  import { onMount } from 'svelte';
  import type { Snippet } from 'svelte';

  type DockSide = 'left' | 'right';
  type DragMode = 'move' | 'resize';
  type Rect = { x: number; y: number; w: number; h: number };

  type Props = {
    title: string;
    side: DockSide;
    width?: number;
    height?: number;
    minWidth?: number;
    minHeight?: number;
    onClose: () => void;
    children?: Snippet;
  };

  let {
    title,
    side,
    width = 360,
    height = 520,
    minWidth = 260,
    minHeight = 220,
    onClose,
    children
  }: Props = $props();

  type DragState = {
    mode: DragMode;
    startX: number;
    startY: number;
    startRect: Rect;
    currentRect: Rect;
  };

  let dockEl: HTMLElement | null = null;
  let rect: Rect = { x: 8, y: 8, w: 360, h: 520 };
  let drag: DragState | null = null;
  let initialized = false;
  let isDragging = $state(false);
  let bounds = { width: 0, height: 0 };
  let pendingRect: Rect | null = null;
  let rafId = 0;

  function updateBounds() {
    const parent = dockEl?.parentElement as HTMLElement | null;
    bounds = {
      width: parent?.clientWidth ?? 0,
      height: parent?.clientHeight ?? 0
    };
  }

  function clampRect(candidate: Rect): Rect {
    if (bounds.width <= 0 || bounds.height <= 0) {
      return candidate;
    }

    const w = Math.min(Math.max(minWidth, candidate.w), Math.max(minWidth, bounds.width - 16));
    const h = Math.min(Math.max(minHeight, candidate.h), Math.max(minHeight, bounds.height - 16));
    const x = Math.max(8, Math.min(bounds.width - w - 8, candidate.x));
    const y = Math.max(8, Math.min(bounds.height - h - 8, candidate.y));
    return { x, y, w, h };
  }

  function applyRectStyle(next: Rect) {
    if (!dockEl) return;
    dockEl.style.transform = `translate3d(${next.x}px, ${next.y}px, 0)`;
    dockEl.style.width = `${next.w}px`;
    dockEl.style.height = `${next.h}px`;
  }

  function scheduleRectUpdate(next: Rect) {
    pendingRect = next;
    if (rafId !== 0) return;
    rafId = requestAnimationFrame(() => {
      rafId = 0;
      if (pendingRect) {
        applyRectStyle(pendingRect);
      }
    });
  }

  function initializeRect() {
    if (initialized) return;
    updateBounds();
    if (bounds.width <= 0 || bounds.height <= 0) return;

    const w = Math.min(width, Math.max(minWidth, bounds.width - 16));
    const h = Math.min(height, Math.max(minHeight, bounds.height - 16));
    const x = side === 'right' ? Math.max(8, bounds.width - w - 8) : 8;

    rect = clampRect({ x, y: 8, w, h });
    applyRectStyle(rect);
    initialized = true;
  }

  function startDrag(event: PointerEvent, mode: DragMode) {
    if (event.button !== 0) return;
    const base = drag?.currentRect ?? rect;
    updateBounds();
    drag = {
      mode,
      startX: event.clientX,
      startY: event.clientY,
      startRect: { ...base },
      currentRect: { ...base }
    };
    isDragging = true;
    event.preventDefault();
    event.stopPropagation();
  }

  function applyDrag(event: PointerEvent) {
    if (!drag) return;

    const dx = event.clientX - drag.startX;
    const dy = event.clientY - drag.startY;

    if (drag.mode === 'move') {
      const next = clampRect({
        ...drag.startRect,
        x: drag.startRect.x + dx,
        y: drag.startRect.y + dy
      });
      drag.currentRect = next;
      scheduleRectUpdate(next);
      return;
    }

    const next = clampRect({
      ...drag.startRect,
      w: drag.startRect.w + dx,
      h: drag.startRect.h + dy
    });
    drag.currentRect = next;
    scheduleRectUpdate(next);
  }

  function stopDrag() {
    if (drag) {
      rect = drag.currentRect;
      applyRectStyle(rect);
    }
    isDragging = false;
    drag = null;
  }

  onMount(() => {
    initializeRect();
    applyRectStyle(rect);

    const parent = dockEl?.parentElement as HTMLElement | null;
    const observer = new ResizeObserver(() => {
      updateBounds();
      initializeRect();
      if (drag) {
        drag.startRect = clampRect(drag.startRect);
        drag.currentRect = clampRect(drag.currentRect);
        scheduleRectUpdate(drag.currentRect);
        return;
      }
      rect = clampRect(rect);
      applyRectStyle(rect);
    });

    if (parent) {
      observer.observe(parent);
    }

    const onMove = (event: PointerEvent) => applyDrag(event);
    const onUp = () => stopDrag();

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);

    return () => {
      observer.disconnect();
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
      if (rafId !== 0) {
        cancelAnimationFrame(rafId);
      }
    };
  });
</script>

<section class="floating-dock panel" class:dragging={isDragging} bind:this={dockEl}>
  <div
    class="dock-head"
    role="toolbar"
    tabindex="0"
    aria-label={`${title} controls`}
    onpointerdown={(event) => startDrag(event, 'move')}
  >
    <h3>{title}</h3>
    <button
      class="dock-close"
      aria-label={`Close ${title.toLowerCase()}`}
      onpointerdown={(event) => event.stopPropagation()}
      onclick={onClose}
    >
      <FontAwesomeIcon icon={faXmark} />
    </button>
  </div>
  <div class="dock-content">
    {#if children}
      {@render children()}
    {/if}
  </div>
  <button
    class="dock-resize"
    aria-label={`Resize ${title.toLowerCase()}`}
    title="Resize"
    onpointerdown={(event) => startDrag(event, 'resize')}
  >
    <FontAwesomeIcon icon={faUpRightAndDownLeftFromCenter} />
  </button>
</section>

<style>
  .floating-dock {
    position: absolute;
    left: 0;
    top: 0;
    z-index: 30;
    box-shadow: 0 22px 42px rgba(0, 0, 0, 0.38);
    backdrop-filter: blur(3px);
    display: grid;
    grid-template-rows: auto 1fr;
    overflow: hidden;
    min-width: 0;
    min-height: 0;
    contain: layout paint;
    will-change: transform, width, height;
  }

  .floating-dock.dragging .dock-content {
    pointer-events: none;
  }

  .dock-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.4rem;
    padding: 0.34rem 0.42rem;
    border-bottom: 1px solid var(--border-subtle);
    background: #171d28;
    cursor: move;
    user-select: none;
    touch-action: none;
  }

  .dock-head h3 {
    margin: 0;
    font-size: 0.76rem;
    color: var(--text-strong);
    letter-spacing: 0.01em;
  }

  .dock-close {
    width: 1.42rem;
    height: 1.42rem;
    display: inline-grid;
    place-items: center;
    border-radius: 999px;
    border: 1px solid rgba(248, 113, 113, 0.45);
    background: #3a2024;
    color: #fecaca;
    padding: 0;
    line-height: 1;
  }

  .dock-close :global(svg) {
    width: 0.66rem;
    height: 0.66rem;
    display: block;
  }

  .dock-content {
    min-height: 0;
    overflow: auto;
  }

  .dock-content :global(.signal-browser),
  .dock-content :global(.inspector) {
    height: 100%;
    border: none;
    border-radius: 0;
    box-shadow: none;
  }

  .dock-content :global(.signal-browser > header),
  .dock-content :global(.inspector > header) {
    display: none;
  }

  .dock-resize {
    position: absolute;
    right: 0.14rem;
    bottom: 0.02rem;
    width: 0.9rem;
    height: 0.9rem;
    display: inline-grid;
    place-items: center;
    border: none;
    background: transparent;
    color: var(--text-soft);
    cursor: nwse-resize;
    line-height: 1;
    padding: 0;
    user-select: none;
    touch-action: none;
  }

  .dock-resize :global(svg) {
    width: 0.52rem;
    height: 0.52rem;
    display: block;
  }
</style>
