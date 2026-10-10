<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { formatNumber } from '../../../lib/format';
  import { finiteSetting } from '../../../lib/board/widgets/controls-numeric';
  import { alertTone, arcPath, fraction, gaugeAngle } from '../../../lib/board/widgets/controls-values';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const value = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null;
  });
  const min = $derived(finiteSetting(ctx.setting<number>('min')) ?? 0);
  const max = $derived(finiteSetting(ctx.setting<number>('max')) ?? 100);
  const alertWhen = $derived(ctx.setting<string>('alertWhen') === 'low' ? 'low' : 'high');
  const warn = $derived(ctx.setting<number>('warn'));
  const fault = $derived(ctx.setting<number>('fault'));
  const tone = $derived(alertTone(value, { warn, fault, alertWhen }));
  const unit = $derived(signal?.unit ?? '');
  const angle = $derived(gaugeAngle(value, min, max));

  const CX = 60;
  const CY = 60;
  const R = 46;
  const warnFrac = $derived(finiteSetting(warn) === null ? null : fraction(warn, min, max));
  const faultFrac = $derived(finiteSetting(fault) === null ? null : fraction(fault, min, max));
  // Threshold bands: from the threshold to the far end of the sweep for "high", from the start to it for "low".
  const band = (f: number | null) =>
    f === null ? '' : alertWhen === 'high' ? arcPath(CX, CY, R, f, 1) : arcPath(CX, CY, R, 0, f);
  const valueArc = $derived(
    value === null ? '' : alertWhen === 'high' ? arcPath(CX, CY, R, 0, fraction(value, min, max)) : arcPath(CX, CY, R, fraction(value, min, max), 1)
  );
  const toneColour = $derived(tone === 'fault' ? 'var(--fault)' : tone === 'warn' ? 'var(--warn)' : 'var(--info)');
</script>

<div class="gauge" data-tone={tone ?? 'ok'}>
  <svg viewBox="0 0 120 92" role="img" aria-label="{signal?.name ?? 'Gauge'}: {value === null ? 'no value' : formatNumber(value)}">
    <path d={arcPath(CX, CY, R, 0, 1)} class="track" fill="none" />
    {#if warnFrac !== null}<path d={band(warnFrac)} class="band warn" fill="none" />{/if}
    {#if faultFrac !== null}<path d={band(faultFrac)} class="band fault" fill="none" />{/if}
    {#if value !== null}<path d={valueArc} class="needle-arc" style="stroke: {toneColour}" fill="none" />{/if}
    <line x1={CX} y1={CY} x2={CX} y2={CY - R + 6} class="needle" class:idle={value === null} transform="rotate({value === null ? 0 : angle} {CX} {CY})" />
    <circle cx={CX} cy={CY} r="4" class="hub" />
    <text x="8" y="80" class="scale">{formatNumber(min)}</text>
    <text x="112" y="80" class="scale" text-anchor="end">{formatNumber(max)}</text>
  </svg>
  <div class="readout">
    <span class="value ui-mono">{value === null ? '--' : formatNumber(value)}{#if unit}<small>{unit}</small>{/if}</span>
    <span class="label">{signal?.name ?? 'Gauge'}</span>
  </div>
</div>

<style>
  .gauge {
    position: relative;
    display: grid;
    place-items: center;
    height: 100%;
    padding: 4px;
    container-type: size;
  }
  svg {
    width: 100%;
    height: 100%;
    max-height: 100cqh;
    overflow: visible;
  }
  .track {
    stroke: var(--surface-3);
    stroke-width: 8;
    stroke-linecap: round;
  }
  .band {
    stroke-width: 8;
    opacity: 0.45;
  }
  .band.warn {
    stroke: var(--warn);
  }
  .band.fault {
    stroke: var(--fault);
  }
  .needle-arc {
    stroke-width: 8;
    stroke-linecap: round;
  }
  .needle {
    stroke: var(--text-strong);
    stroke-width: 2.5;
    stroke-linecap: round;
    transition: transform 250ms ease-out;
  }
  .needle.idle {
    opacity: 0.25;
  }
  .hub {
    fill: var(--text-strong);
  }
  .scale {
    fill: var(--text-faint);
    font-size: 8px;
    font-family: var(--font-mono);
  }
  .readout {
    position: absolute;
    left: 0;
    right: 0;
    bottom: 6%;
    display: grid;
    justify-items: center;
    gap: 0;
    pointer-events: none;
  }
  .value {
    font-size: clamp(14px, 22cqh, 36px);
    font-weight: 600;
    line-height: 1.1;
    color: var(--text-strong);
    white-space: nowrap;
  }
  .gauge[data-tone='warn'] .value {
    color: var(--warn);
  }
  .gauge[data-tone='fault'] .value {
    color: var(--fault);
  }
  .value small {
    margin-left: 3px;
    font-size: 0.45em;
    font-weight: 500;
    color: var(--text-faint);
  }
  .label {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    max-width: 90%;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
