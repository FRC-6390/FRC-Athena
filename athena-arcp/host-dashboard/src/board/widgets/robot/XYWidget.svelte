<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { alignPairs, decimateStride, newestTimestamp, numericOf, xyDomain } from '../../../lib/board/widgets/robot-series';
  import { prepareCanvas, readPalette, shortNumber } from '../../../lib/board/widgets/robot-canvas';

  let { ctx }: { ctx: WidgetContext } = $props();

  const xSignal = $derived(ctx.bindings.x?.[0]);
  const ySignal = $derived(ctx.bindings.y?.[0]);
  const windowSeconds = $derived(Math.max(1, ctx.setting<number>('window')));
  const equal = $derived(ctx.setting<boolean>('equal'));
  const trail = $derived(ctx.setting<boolean>('trail'));

  /** Paired points for the window, decimated so the canvas work stays bounded on long histories. */
  const model = $derived.by(() => {
    void ctx.live.current;
    if (!xSignal || !ySignal) return null;
    const xs = ctx.samples(xSignal);
    const ys = ctx.samples(ySignal);
    const now = newestTimestamp([xs, ys]);
    const all = alignPairs(xs, ys, now, windowSeconds);
    return { points: decimateStride(all, 500), count: all.length };
  });
  const latestX = $derived(numericOf(ctx.latest(xSignal)));
  const latestY = $derived(numericOf(ctx.latest(ySignal)));

  let canvas = $state<HTMLCanvasElement | undefined>(undefined);
  const PAD = { left: 34, right: 8, top: 8, bottom: 20 };

  $effect(() => {
    const el = canvas;
    const width = ctx.size.width;
    const height = ctx.size.height;
    const data = model;
    const eq = equal;
    const showTrail = trail;
    if (!el || width < 40 || height < 40) return;
    const g = prepareCanvas(el, width, height);
    if (!g) return;
    const palette = readPalette();
    const plotW = width - PAD.left - PAD.right;
    const plotH = height - PAD.top - PAD.bottom;
    const points = data?.points ?? [];
    const domain = xyDomain(points, eq);
    const sx = (x: number) => PAD.left + ((x - domain.x.min) / (domain.x.max - domain.x.min)) * plotW;
    const sy = (y: number) => PAD.top + (1 - (y - domain.y.min) / (domain.y.max - domain.y.min)) * plotH;

    // Grid and axis labels: four divisions per axis.
    g.font = '10px ui-monospace, monospace';
    g.lineWidth = 1;
    g.strokeStyle = palette.border;
    g.fillStyle = palette.textFaint;
    for (let i = 0; i <= 4; i += 1) {
      const f = i / 4;
      const gy = PAD.top + f * plotH;
      const gx = PAD.left + f * plotW;
      g.beginPath();
      g.moveTo(PAD.left, gy + 0.5);
      g.lineTo(PAD.left + plotW, gy + 0.5);
      g.moveTo(gx + 0.5, PAD.top);
      g.lineTo(gx + 0.5, PAD.top + plotH);
      g.stroke();
      g.textAlign = 'right';
      g.textBaseline = 'middle';
      g.fillText(shortNumber(domain.y.max - f * (domain.y.max - domain.y.min)), PAD.left - 4, gy);
      g.textAlign = 'center';
      g.textBaseline = 'top';
      g.fillText(shortNumber(domain.x.min + f * (domain.x.max - domain.x.min)), gx, PAD.top + plotH + 4);
    }

    if (points.length === 0) {
      g.fillStyle = palette.textFaint;
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText(xSignal && ySignal ? 'no paired samples in window' : 'bind X and Y', PAD.left + plotW / 2, PAD.top + plotH / 2);
      return;
    }

    // Trail: older points fade out, the newest point is drawn as a solid dot.
    g.lineWidth = 1.6;
    g.lineJoin = 'round';
    g.strokeStyle = palette.series[0];
    const n = points.length;
    if (showTrail && n > 1) {
      for (let i = 1; i < n; i += 1) {
        g.globalAlpha = 0.15 + (0.85 * i) / (n - 1);
        g.beginPath();
        g.moveTo(sx(points[i - 1].x), sy(points[i - 1].y));
        g.lineTo(sx(points[i].x), sy(points[i].y));
        g.stroke();
      }
      g.globalAlpha = 1;
    } else {
      g.globalAlpha = 1;
      g.fillStyle = palette.series[0];
      for (const p of points) {
        g.beginPath();
        g.arc(sx(p.x), sy(p.y), 1.6, 0, Math.PI * 2);
        g.fill();
      }
    }
    const last = points[n - 1];
    g.fillStyle = palette.series[2];
    g.beginPath();
    g.arc(sx(last.x), sy(last.y), 4, 0, Math.PI * 2);
    g.fill();
  });
</script>

<div class="xy">
  <canvas bind:this={canvas}></canvas>
  <div class="readout ui-mono" title={xSignal && ySignal ? `${xSignal.path} / ${ySignal.path}` : ''}>
    <span>x {latestX === null ? '--' : shortNumber(latestX)}{xSignal?.unit ? ` ${xSignal.unit}` : ''}</span>
    <span>y {latestY === null ? '--' : shortNumber(latestY)}{ySignal?.unit ? ` ${ySignal.unit}` : ''}</span>
  </div>
</div>

<style>
  .xy {
    position: relative;
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  canvas {
    display: block;
  }
  .readout {
    position: absolute;
    top: 6px;
    right: 8px;
    display: flex;
    gap: 10px;
    font-size: var(--fs-xs);
    color: var(--text-strong);
    pointer-events: none;
    background: transparent;
  }
</style>
