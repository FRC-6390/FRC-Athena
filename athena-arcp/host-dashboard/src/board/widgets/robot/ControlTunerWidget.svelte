<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { parseInput, formatNumber } from '../../../lib/format';
  import { controlSiblings } from '../../../lib/board/widgets/robot-siblings';
  import {
    decimateStride,
    newestTimestamp,
    paddedDomain,
    windowedNumeric,
    type TimedValue
  } from '../../../lib/board/widgets/robot-series';
  import { prepareCanvas, readPalette } from '../../../lib/board/widgets/robot-canvas';

  let { ctx }: { ctx: WidgetContext } = $props();

  const anchor = $derived(ctx.bindings.signal?.[0]);
  const windowSeconds = $derived(Math.max(1, ctx.setting<number>('window')));

  const ctrl = $derived.by(() => {
    void ctx.live.current;
    if (!anchor) return null;
    return controlSiblings(ctx.session.store.allSignals(), anchor.path);
  });

  type Trace = { key: 'target' | 'reference' | 'measured'; label: string; signal: SignalDescriptor | null; color: string; points: TimedValue[] };

  /** Target, reference and measured traces over the window, decimated for drawing. */
  const traces = $derived.by((): Trace[] => {
    void ctx.live.current;
    const c = ctrl;
    const defs = [
      { key: 'target', label: 'target', signal: c?.target ?? null, color: 'var(--series-target)' },
      { key: 'reference', label: 'reference', signal: c?.reference ?? null, color: 'var(--series-reference)' },
      { key: 'measured', label: 'measured', signal: c?.measured ?? null, color: 'var(--series-measured)' }
    ] as const;
    const lists = defs.map((d) => (d.signal ? ctx.samples(d.signal) : []));
    const now = newestTimestamp(lists);
    return defs.map((d, i) => ({
      key: d.key,
      label: d.label,
      signal: d.signal,
      color: d.color,
      points: decimateStride(windowedNumeric(lists[i], now, windowSeconds), 300)
    }));
  });

  const hasTraces = $derived(traces.some((t) => t.points.length > 0));
  const velocity = $derived.by(() => {
    void ctx.live.current;
    const s = ctrl?.velocity;
    if (!s) return null;
    const v = ctx.latest(s);
    return typeof v === 'number' ? v : null;
  });

  let canvas = $state<HTMLCanvasElement | undefined>(undefined);
  $effect(() => {
    const el = canvas;
    const width = ctx.size.width;
    const height = ctx.size.height;
    const data = traces;
    const seconds = windowSeconds;
    if (!el || width < 60 || height < 40) return;
    const g = prepareCanvas(el, width, height);
    if (!g) return;
    const palette = readPalette();
    const PAD = { l: 6, r: 6, t: 6, b: 16 };
    const plotW = width - PAD.l - PAD.r;
    const plotH = height - PAD.t - PAD.b;
    const values = data.flatMap((t) => t.points.map((p) => p.v));
    const domain = values.length
      ? paddedDomain(Math.min(...values), Math.max(...values), { minSpan: 0.5 })
      : { min: -1, max: 1 };
    const sx = (t: number) => PAD.l + ((t + seconds) / seconds) * plotW;
    const sy = (v: number) => PAD.t + (1 - (v - domain.min) / (domain.max - domain.min || 1)) * plotH;

    g.strokeStyle = palette.border;
    g.lineWidth = 1;
    g.beginPath();
    g.moveTo(PAD.l, PAD.t + plotH + 0.5);
    g.lineTo(PAD.l + plotW, PAD.t + plotH + 0.5);
    g.stroke();
    g.font = '10px ui-monospace, monospace';
    g.fillStyle = palette.textFaint;
    g.textBaseline = 'top';
    g.textAlign = 'left';
    g.fillText(`-${seconds} s`, PAD.l, PAD.t + plotH + 3);
    g.textAlign = 'right';
    g.fillText('now', PAD.l + plotW, PAD.t + plotH + 3);

    if (!values.length) {
      g.textAlign = 'center';
      g.textBaseline = 'middle';
      g.fillText('no traces in window', width / 2, PAD.t + plotH / 2);
      return;
    }
    const names = { target: palette.series[2], reference: palette.series[1], measured: palette.series[0] };
    for (const t of data) {
      if (t.points.length < 2) continue;
      g.strokeStyle = names[t.key];
      g.lineWidth = t.key === 'measured' ? 1.8 : 1.4;
      g.lineJoin = 'round';
      g.setLineDash(t.key === 'reference' ? [4, 3] : []);
      g.beginPath();
      t.points.forEach((p, i) => (i === 0 ? g.moveTo(sx(p.t), sy(p.v)) : g.lineTo(sx(p.t), sy(p.v))));
      g.stroke();
    }
    g.setLineDash([]);
  });

  // Gains: click a value to edit. The robot's reply (accepted or rejected) comes back through the store.
  let editingPath = $state<string | null>(null);
  let draft = $state('');

  async function commit(signal: SignalDescriptor) {
    const parsed = parseInput(signal.type, draft);
    editingPath = null;
    if (parsed !== null) await ctx.write(signal, parsed);
  }

  function rejected(signal: SignalDescriptor): string | null {
    void ctx.live.current;
    const r = ctx.reply(signal);
    return r && r.op === 'write' && !r.accepted ? r.reason : null;
  }

  const fmt = (v: unknown) => (typeof v === 'number' ? formatNumber(v) : '--');
</script>

{#if !anchor}
  <div class="problem">Bind a Controls/ signal</div>
{:else if !ctrl}
  <div class="problem">{anchor.path} is not under a Controls/ group</div>
{:else}
  <div class="tuner">
    <header>
      <span class="name" title={ctrl.root}>{ctrl.name}</span>
      {#if velocity !== null}<span class="vel ui-mono">vel {formatNumber(velocity)}</span>{/if}
    </header>

    <div class="plot" class:empty={!hasTraces}>
      <canvas bind:this={canvas}></canvas>
    </div>

    <div class="legend ui-mono">
      {#each traces as t (t.key)}
        <span class="item"><i style="background:var(--series-{t.key === 'target' ? 'target' : t.key === 'reference' ? 'reference' : 'measured'})"></i>{t.label}<b>{fmt(t.signal ? ctx.latest(t.signal) : undefined)}</b></span>
      {/each}
    </div>

    {#if ctrl.gains.length > 0}
      <div class="gains">
        {#each ctrl.gains as gain (gain.path)}
          {@const err = rejected(gain)}
          {@const locked = ctx.locked(gain)}
          <div class="gain">
            <span class="gname ui-mono" title={gain.path}>{gain.name}</span>
            {#if editingPath === gain.path}
              <!-- svelte-ignore a11y_autofocus -->
              <input
                class="ui-input"
                autofocus
                bind:value={draft}
                onkeydown={(e) => {
                  if (e.key === 'Enter') void commit(gain);
                  if (e.key === 'Escape') editingPath = null;
                }}
                onblur={() => (editingPath = null)}
              />
            {:else}
              <button
                type="button"
                class="gvalue ui-mono"
                disabled={!gain.writable || ctx.editing}
                title={gain.writable ? 'Click to edit' : gain.path}
                onclick={() => {
                  draft = String(ctx.latest(gain) ?? '');
                  editingPath = gain.path;
                }}>{fmt(ctx.latest(gain))}</button
              >
            {/if}
            {#if locked}<span class="ui-badge" data-tone="locked">locked</span>{/if}
            {#if err}<span class="rejected" title={err}>rejected</span>{/if}
          </div>
        {/each}
      </div>
    {/if}
  </div>
{/if}

<style>
  .tuner {
    display: flex;
    flex-direction: column;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    box-sizing: border-box;
    min-height: 0;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
  }
  .name {
    font-size: var(--fs-md);
    font-weight: 600;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .vel {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .plot {
    position: relative;
    flex: 1;
    min-height: 60px;
    border-radius: var(--radius);
    background: var(--surface-2);
    overflow: hidden;
  }
  .plot canvas {
    position: absolute;
    inset: 0;
    display: block;
  }
  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 4px 12px;
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .item {
    display: inline-flex;
    align-items: center;
    gap: 5px;
  }
  .item i {
    width: 10px;
    height: 2px;
  }
  .item b {
    margin-left: 2px;
    font-weight: 500;
    color: var(--text-strong);
  }
  .gains {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(96px, 1fr));
    gap: 4px 8px;
    max-height: 90px;
    overflow: auto;
  }
  .gain {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
  }
  .gname {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    min-width: 28px;
  }
  .gvalue {
    border: none;
    background: transparent;
    padding: 0 2px;
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
    cursor: pointer;
  }
  .gvalue:disabled {
    cursor: default;
    color: var(--text-soft);
  }
  .gain input {
    width: 72px;
    height: 22px;
    font-size: var(--fs-xs);
  }
  .rejected {
    font-size: var(--fs-xs);
    color: var(--fault);
  }
  .problem {
    display: grid;
    place-items: center;
    height: 100%;
    padding: 8px;
    text-align: center;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
