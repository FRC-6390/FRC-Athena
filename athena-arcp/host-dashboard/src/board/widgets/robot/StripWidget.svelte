<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import {
    minMaxColumns,
    newestTimestamp,
    paddedDomain,
    windowedNumeric,
    type TimedValue
  } from '../../../lib/board/widgets/robot-series';
  import { prepareCanvas, readPalette, shortNumber } from '../../../lib/board/widgets/robot-canvas';

  let { ctx }: { ctx: WidgetContext } = $props();

  const series = $derived(ctx.bindings.series ?? []);
  const windowSeconds = $derived(Math.max(1, ctx.setting<number>('window')));
  const LABEL_W = 92;
  const AXIS_H = 18;

  /** Each lane's numeric history, reduced to min/max/last per pixel column. */
  const lanes = $derived.by(() => {
    void ctx.live.current;
    const lists = series.map((s) => ctx.samples(s));
    const now = newestTimestamp(lists);
    return series.map((s, i) => {
      const points: TimedValue[] = windowedNumeric(lists[i], now, windowSeconds);
      return { signal: s, points, latest: ctx.latest(s) };
    });
  });

  let canvas = $state<HTMLCanvasElement | undefined>(undefined);

  $effect(() => {
    const el = canvas;
    const width = ctx.size.width;
    const height = ctx.size.height;
    const data = lanes;
    const seconds = windowSeconds;
    if (!el || width < 60 || height < 30) return;
    const g = prepareCanvas(el, width, height);
    if (!g) return;
    const palette = readPalette();
    const labelW = Math.min(LABEL_W, Math.max(40, width * 0.35));
    const plotX0 = labelW;
    const plotW = Math.max(10, width - labelW - 6);
    const plotH = Math.max(0, height - AXIS_H);
    const laneCount = Math.max(1, data.length);
    const laneH = plotH / laneCount;
    const columns = Math.max(1, Math.floor(plotW));

    if (data.length === 0) {
      g.fillStyle = palette.textFaint;
      g.font = '12px ui-sans-serif, system-ui, sans-serif';
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText('bind one or more numbers', width / 2, height / 2);
      return;
    }

    data.forEach((lane, i) => {
      const top = i * laneH;
      const color = palette.series[i % palette.series.length];
      // Lane separator and label column.
      g.strokeStyle = palette.border;
      g.lineWidth = 1;
      if (i > 0) {
        g.beginPath();
        g.moveTo(0, top + 0.5);
        g.lineTo(width, top + 0.5);
        g.stroke();
      }
      const cols = minMaxColumns(lane.points, columns, -seconds, 0);
      const mid = cols.length ? (Math.min(...cols.map((c) => c.min)) + Math.max(...cols.map((c) => c.max))) / 2 : 0;
      const domain = cols.length
        ? paddedDomain(Math.min(...cols.map((c) => c.min)), Math.max(...cols.map((c) => c.max)), {
            minSpan: Math.max(Math.abs(mid) * 0.1, 0.5)
          })
        : { min: -1, max: 1 };
      const inner = laneH - 6;
      const y = (v: number) => top + 3 + (1 - (v - domain.min) / (domain.max - domain.min || 1)) * inner;

      // Traces: a vertical min..max stroke per column plus the last value connecting the columns.
      if (cols.length > 0) {
        g.strokeStyle = color;
        g.lineWidth = 1.2;
        g.globalAlpha = 0.35;
        for (const c of cols) {
          if (c.max - c.min < 1e-12) continue;
          g.beginPath();
          g.moveTo(plotX0 + c.col + 0.5, y(c.max));
          g.lineTo(plotX0 + c.col + 0.5, y(c.min));
          g.stroke();
        }
        g.globalAlpha = 1;
        g.lineWidth = 1.5;
        g.lineJoin = 'round';
        g.beginPath();
        cols.forEach((c, k) => {
          const px = plotX0 + c.col + 0.5;
          if (k === 0) g.moveTo(px, y(c.last));
          else g.lineTo(px, y(c.last));
        });
        g.stroke();
      }

      // Label column: name, then latest value in the lane colour.
      g.fillStyle = palette.textSoft;
      g.font = '11px ui-sans-serif, system-ui, sans-serif';
      g.textAlign = 'left';
      g.textBaseline = 'top';
      g.fillText(lane.signal.name, 6, top + 5, labelW - 10);
      const value = typeof lane.latest === 'number' ? formatNumber(lane.latest) : '--';
      g.font = '600 13px ui-monospace, monospace';
      g.fillStyle = palette.text;
      g.fillText(`${value}${lane.signal.unit ? ' ' + lane.signal.unit : ''}`, 6, top + 21, labelW - 10);
      if (cols.length === 0) {
        g.fillStyle = palette.textFaint;
        g.textBaseline = 'middle';
        g.fillText('no data', plotX0 + plotW / 2 - 20, top + laneH / 2);
      } else {
        // Domain labels at the right edge.
        g.font = '10px ui-monospace, monospace';
        g.fillStyle = palette.textFaint;
        g.textAlign = 'right';
        g.textBaseline = 'top';
        g.fillText(shortNumber(domain.max), width - 4, top + 3);
        g.textBaseline = 'bottom';
        g.fillText(shortNumber(domain.min), width - 4, top + laneH - 3);
      }
    });

    // Shared time axis.
    g.strokeStyle = palette.border;
    g.beginPath();
    g.moveTo(0, plotH + 0.5);
    g.lineTo(width, plotH + 0.5);
    g.stroke();
    g.font = '10px ui-monospace, monospace';
    g.fillStyle = palette.textFaint;
    g.textBaseline = 'top';
    g.textAlign = 'left';
    g.fillText(`-${seconds} s`, plotX0, plotH + 4);
    g.textAlign = 'center';
    g.fillText(`-${Math.round(seconds / 2)} s`, plotX0 + plotW / 2, plotH + 4);
    g.textAlign = 'right';
    g.fillText('now', plotX0 + plotW, plotH + 4);
  });
</script>

<div class="strip">
  <canvas bind:this={canvas}></canvas>
</div>

<style>
  .strip {
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  canvas {
    display: block;
  }
</style>
