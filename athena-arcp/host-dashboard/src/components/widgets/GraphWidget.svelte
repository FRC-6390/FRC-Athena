<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readGraphConfig, type GraphSeriesStyle } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signalById: Map<number, SignalRow>;
    historyBySignal: Map<number, number[]>;
    historyTimeBySignal: Map<number, number[]>;
    nowMs: number;
    configRaw?: WidgetConfigRecord;
  };

  type SeriesRender = {
    signalId: number;
    label: string;
    color: string;
    role: string;
    style: GraphSeriesStyle;
    values: number[];
    timeMs: number[];
    dotStartIndex: number;
    latest: number | null;
    latestTimestampMs: number | null;
  };

  type TimeTick = {
    key: string;
    x: number;
    label: string;
  };

  let { signal, signalById, historyBySignal, historyTimeBySignal, nowMs, configRaw }: Props = $props();
  const GRAPH_TIME_WINDOW_MS = 10_000;
  const STALE_THRESHOLD_MIN_MS = 1200;

  function numericFromSignal(value: string): number | null {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function yFor(value: number, min: number, max: number): number {
    const range = Math.max(1e-9, max - min);
    const ratio = (value - min) / range;
    return 34 - ratio * 30;
  }

  function xForTime(timeMs: number, windowStartMs: number, windowEndMs: number): number {
    const span = Math.max(1, windowEndMs - windowStartMs);
    const ratio = (timeMs - windowStartMs) / span;
    return Math.max(0, Math.min(1, ratio)) * 100;
  }

  function formatWindowLabel(ms: number): string {
    if (ms < 1000) return `${Math.round(ms)}ms window`;
    if (ms < 10000) return `${(ms / 1000).toFixed(1)}s window`;
    return `${Math.round(ms / 1000)}s window`;
  }

  function linePath(
    values: number[],
    timeMs: number[],
    min: number,
    max: number,
    windowStartMs: number,
    windowEndMs: number
  ): string {
    if (values.length === 0 || timeMs.length === 0) return 'M 0 18 L 100 18';
    return values
      .map((value, index) => {
        const x = xForTime(timeMs[index] ?? windowEndMs, windowStartMs, windowEndMs);
        const y = yFor(value, min, max);
        return `${index === 0 ? 'M' : 'L'} ${x.toFixed(2)} ${y.toFixed(2)}`;
      })
      .join(' ');
  }

  function stepPath(
    values: number[],
    timeMs: number[],
    min: number,
    max: number,
    windowStartMs: number,
    windowEndMs: number
  ): string {
    if (values.length === 0 || timeMs.length === 0) return 'M 0 18 L 100 18';
    const points = values.map((value, index) => {
      return {
        x: xForTime(timeMs[index] ?? windowEndMs, windowStartMs, windowEndMs),
        y: yFor(value, min, max)
      };
    });

    let path = `M ${points[0].x.toFixed(2)} ${points[0].y.toFixed(2)}`;
    for (let i = 1; i < points.length; i++) {
      path += ` L ${points[i].x.toFixed(2)} ${points[i - 1].y.toFixed(2)}`;
      path += ` L ${points[i].x.toFixed(2)} ${points[i].y.toFixed(2)}`;
    }
    return path;
  }

  const computed = $derived.by(() => {
    const config = readGraphConfig(configRaw, signal);

    const series = config.series
      .map((entry): SeriesRender | null => {
        const boundSignal = signalById.get(entry.signalId);
        let samples = [...(historyBySignal.get(entry.signalId) ?? [])];
        let sampleTimes = [...(historyTimeBySignal.get(entry.signalId) ?? [])];

        if (sampleTimes.length > samples.length) {
          sampleTimes = sampleTimes.slice(sampleTimes.length - samples.length);
        } else if (samples.length > sampleTimes.length) {
          if (sampleTimes.length === 0) {
            const stepMs = 200;
            sampleTimes = samples.map((_, index) => nowMs - (samples.length - index - 1) * stepMs);
          } else {
            const stepMs =
              sampleTimes.length > 1
                ? Math.max(
                    1,
                    (sampleTimes[sampleTimes.length - 1] - sampleTimes[0]) /
                      (sampleTimes.length - 1)
                  )
                : 200;
            const missing = samples.length - sampleTimes.length;
            const first = sampleTimes[0];
            const prefix = Array.from(
              { length: missing },
              (_, index) => Math.max(0, Math.round(first - stepMs * (missing - index)))
            );
            sampleTimes = [...prefix, ...sampleTimes];
          }
        }

        if (samples.length === 0 && boundSignal) {
          const current = numericFromSignal(boundSignal.value);
          if (current !== null) {
            samples.push(current);
            sampleTimes.push(nowMs);
          }
        }

        const latest = samples.length > 0 ? samples[samples.length - 1] ?? null : null;
        const latestTimestampMs =
          sampleTimes.length > 0 ? sampleTimes[sampleTimes.length - 1] ?? null : null;

        return {
          signalId: entry.signalId,
          label: entry.label,
          color: entry.color,
          role: entry.role,
          style: entry.style,
          values: samples,
          timeMs: sampleTimes,
          dotStartIndex: Math.max(0, samples.length - 18),
          latest,
          latestTimestampMs
        };
      })
      .filter((entry): entry is SeriesRender => entry !== null);

    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    let latestSampleMs = Number.NEGATIVE_INFINITY;
    const intervals: number[] = [];

    for (const entry of series) {
      for (const value of entry.values) {
        if (value < min) min = value;
        if (value > max) max = value;
      }

      for (let index = 1; index < entry.timeMs.length; index++) {
        const delta = entry.timeMs[index] - entry.timeMs[index - 1];
        if (delta > 0 && delta < 60000) {
          intervals.push(delta);
        }
      }

      if (entry.timeMs.length > 0) {
        const last = entry.timeMs[entry.timeMs.length - 1];
        if (last > latestSampleMs) latestSampleMs = last;
      }
    }

    if (config.yMin !== null) min = config.yMin;
    if (config.yMax !== null) max = config.yMax;

    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      min = 0;
      max = 1;
    }
    if (max <= min) {
      max = min + 1;
    }

    intervals.sort((left, right) => left - right);
    const estimatedIntervalMs =
      intervals.length > 0
        ? intervals[Math.floor(intervals.length / 2)]
        : 200;

    const newestSampleMs = Number.isFinite(latestSampleMs) ? latestSampleMs : null;
    const windowSpanMs = GRAPH_TIME_WINDOW_MS;
    const windowEndMs = newestSampleMs ?? nowMs;
    const windowStartMs = windowEndMs - windowSpanMs;

    const tickCount = 6;
    const xTicks: TimeTick[] = Array.from({ length: tickCount }, (_, index) => {
      const ratio = tickCount > 1 ? index / (tickCount - 1) : 0;
      const ageMs = windowSpanMs * (1 - ratio);
      return {
        key: `tick-${index}-${Math.round(ageMs)}`,
        x: ratio * 100,
        label: index === tickCount - 1 ? 'now' : `-${Math.round(ageMs / 1000)}s`
      };
    });

    const lastSampleAgeMs = newestSampleMs === null ? null : Math.max(0, nowMs - newestSampleMs);
    const staleThresholdMs = Math.max(STALE_THRESHOLD_MIN_MS, estimatedIntervalMs * 2.5);
    const streaming = lastSampleAgeMs !== null && lastSampleAgeMs <= staleThresholdMs;
    const streamLabel =
      lastSampleAgeMs === null
        ? 'NO DATA'
        : streaming
          ? 'LIVE'
          : 'STALE';
    const lastSampleX =
      newestSampleMs === null ? null : xForTime(newestSampleMs, windowStartMs, windowEndMs);

    return {
      config,
      series,
      min,
      max,
      windowStartMs,
      windowEndMs,
      xTicks,
      streamLabel,
      streaming,
      windowLabel: formatWindowLabel(windowSpanMs),
      lastSampleX
    };
  });
</script>

{#if computed.series.length === 0}
  <div class="graph-empty">No numeric series configured.</div>
{:else}
  <div class="graph-root">
    <svg viewBox="0 0 100 36" preserveAspectRatio="none" aria-label="graph widget">
      <line class="guide" x1="0" y1="4" x2="100" y2="4" />
      <line class="guide" x1="0" y1="18" x2="100" y2="18" />
      <line class="guide" x1="0" y1="32" x2="100" y2="32" />
      {#each computed.xTicks as tick (tick.key)}
        <line class="guide x-guide" x1={tick.x} y1="2" x2={tick.x} y2="34" />
      {/each}
      {#if computed.lastSampleX !== null}
        <line class="last-sample" x1={computed.lastSampleX} y1="2" x2={computed.lastSampleX} y2="34" />
      {/if}

      {#each computed.series as series (series.signalId)}
        <path
          d={
            series.style === 'step'
              ? stepPath(
                  series.values,
                  series.timeMs,
                  computed.min,
                  computed.max,
                  computed.windowStartMs,
                  computed.windowEndMs
                )
              : linePath(
                  series.values,
                  series.timeMs,
                  computed.min,
                  computed.max,
                  computed.windowStartMs,
                  computed.windowEndMs
                )
          }
          stroke={series.color}
          class={`series ${series.style}`}
        />

        {#if series.style === 'dot'}
          {#each series.values.slice(series.dotStartIndex) as value, index (`${series.signalId}-${index}`)}
            {@const sampleIndex = series.dotStartIndex + index}
            {@const sampleTime = series.timeMs[sampleIndex] ?? computed.windowEndMs}
            <circle
              cx={xForTime(sampleTime, computed.windowStartMs, computed.windowEndMs)}
              cy={yFor(value, computed.min, computed.max)}
              r="0.85"
              fill={series.color}
            />
          {/each}
        {/if}
      {/each}
    </svg>

    <div class="time-row">
      {#each computed.xTicks as tick (tick.key)}
        <span>{tick.label}</span>
      {/each}
    </div>

    <div class="range-row">
      <span>{computed.min.toFixed(2)}</span>
      <span class={`stream-state ${computed.streaming ? 'live' : 'stale'}`}>
        {computed.streamLabel} | {computed.windowLabel}
      </span>
      <span>{computed.max.toFixed(2)}</span>
    </div>

    {#if computed.config.showLegend}
      <div class="legend">
        {#each computed.series as series (series.signalId)}
          <span class="legend-item">
            <i style={`background:${series.color};`}></i>
            <strong>{series.label}</strong>
            <em>{series.role}</em>
            <b>{series.latest === null ? 'n/a' : series.latest.toFixed(3)}</b>
          </span>
        {/each}
      </div>
    {/if}
  </div>
{/if}

<style>
  .graph-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto auto auto;
    gap: 0.22rem;
  }

  .graph-empty {
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 2.4rem;
    border-radius: 6px;
    background: #1b2534;
    border: 1px solid rgba(99, 115, 140, 0.48);
  }

  .guide {
    stroke: rgba(153, 164, 180, 0.16);
    stroke-width: 0.45;
  }

  .x-guide {
    stroke: rgba(153, 164, 180, 0.12);
    stroke-dasharray: 1 1.8;
  }

  .last-sample {
    stroke: rgba(180, 35, 45, 0.8);
    stroke-width: 0.5;
    vector-effect: non-scaling-stroke;
  }

  .series {
    fill: none;
    stroke-width: 1.2;
    vector-effect: non-scaling-stroke;
  }

  .series.dot {
    stroke-opacity: 0.45;
    stroke-dasharray: 1 1.5;
  }

  .time-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    color: var(--text-soft);
    font-family: var(--font-mono);
    font-size: 0.6rem;
    line-height: 1;
    user-select: none;
  }

  .range-row {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.2rem;
    color: var(--text-soft);
    font-family: var(--font-mono);
    font-size: 0.62rem;
    line-height: 1;
  }

  .range-row span:nth-child(2) {
    text-align: center;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .stream-state.live {
    color: #bbf7d0;
  }

  .stream-state.stale {
    color: #fca5a5;
  }

  .legend {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
    gap: 0.2rem 0.34rem;
  }

  .legend-item {
    display: inline-grid;
    grid-template-columns: auto minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.2rem 0.34rem;
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: var(--surface-3);
    padding: 0.15rem 0.22rem;
    font-size: 0.62rem;
  }

  .legend-item i {
    width: 0.48rem;
    height: 0.48rem;
    border-radius: 999px;
    display: block;
  }

  .legend-item strong {
    min-width: 0;
    color: var(--text);
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .legend-item em {
    color: var(--text-soft);
    font-style: normal;
    text-transform: uppercase;
    letter-spacing: 0.02em;
  }

  .legend-item b {
    grid-column: 2 / 4;
    justify-self: end;
    color: var(--text-strong);
    font-family: var(--font-mono);
    font-weight: 600;
  }
</style>
