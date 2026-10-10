/**
 * Canvas helpers for the robot chart widgets. Canvas cannot read `var(--token)`, so colours are resolved from
 * the document's custom properties at draw time (this keeps light and dark themes working).
 */

/** Resolves a theme token such as `--series-target` to its computed value. Falls back to `fallback`. */
export function themeColor(name: string, fallback = 'CanvasText'): string {
  if (typeof document === 'undefined') return fallback;
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return value === '' ? fallback : value;
}

export type ChartPalette = {
  text: string;
  textSoft: string;
  textFaint: string;
  border: string;
  surface: string;
  info: string;
  ok: string;
  warn: string;
  fault: string;
  series: string[];
};

/** Reads every colour the charts need in one pass. Series colours cycle through `--series-*`. */
export function readPalette(): ChartPalette {
  return {
    text: themeColor('--text-strong'),
    textSoft: themeColor('--text-soft'),
    textFaint: themeColor('--text-faint'),
    border: themeColor('--border'),
    surface: themeColor('--surface-2'),
    info: themeColor('--info'),
    ok: themeColor('--ok'),
    warn: themeColor('--warn'),
    fault: themeColor('--fault'),
    series: [
      themeColor('--series-measured'),
      themeColor('--series-reference'),
      themeColor('--series-target'),
      themeColor('--series-4'),
      themeColor('--series-5'),
      themeColor('--text')
    ]
  };
}

/**
 * Sizes a canvas for its CSS box at device pixel ratio and returns a context whose coordinates are CSS pixels.
 * Re-setting `width` clears the canvas, so this is safe to call on every draw.
 */
export function prepareCanvas(canvas: HTMLCanvasElement, width: number, height: number): CanvasRenderingContext2D | null {
  const dpr = typeof window === 'undefined' ? 1 : Math.max(1, window.devicePixelRatio || 1);
  const w = Math.max(1, Math.floor(width));
  const h = Math.max(1, Math.floor(height));
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  const context = canvas.getContext('2d');
  if (!context) return null;
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  context.clearRect(0, 0, w, h);
  return context;
}

/** Compact number label: integers as-is, otherwise up to three significant decimals. */
export function shortNumber(value: number): string {
  if (!Number.isFinite(value)) return '--';
  if (Number.isInteger(value)) return value.toString();
  const abs = Math.abs(value);
  if (abs >= 1000) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  return value.toFixed(2);
}
