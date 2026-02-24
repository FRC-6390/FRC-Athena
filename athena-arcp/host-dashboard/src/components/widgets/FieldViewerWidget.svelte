<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseTrajectoryPoints, readFieldConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let svgEl = $state<SVGSVGElement | null>(null);

  const config = $derived(readFieldConfig(configRaw, signal, signals));

  function numericValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  const pose = $derived.by(() => {
    const x = numericValue(config.xSignalId);
    const y = numericValue(config.ySignalId);
    const heading = numericValue(config.headingSignalId) ?? 0;

    return {
      x,
      y,
      heading
    };
  });

  const trajectory = $derived.by(() => {
    if (config.trajectorySignalId === null) return [];
    const row = signalById.get(config.trajectorySignalId);
    if (!row) return [];
    return parseTrajectoryPoints(row.value);
  });

  const fieldImageUrl = $derived.by(() => {
    const explicit = config.imageUrl.trim();
    if (explicit) return explicit;
    if (config.imageSignalId === null) return '';
    const row = signalById.get(config.imageSignalId);
    return row?.value.trim() ?? '';
  });

  function toFieldY(y: number): number {
    return config.fieldWidth - y;
  }

  function clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }

  function fieldViewport(rect: DOMRect): { left: number; top: number; width: number; height: number } {
    const safeWidth = Math.max(1, rect.width);
    const safeHeight = Math.max(1, rect.height);
    const fieldAspect = Math.max(1e-6, config.fieldLength / config.fieldWidth);
    const viewportAspect = safeWidth / safeHeight;

    if (viewportAspect > fieldAspect) {
      const width = safeHeight * fieldAspect;
      const left = rect.left + (safeWidth - width) / 2;
      return { left, top: rect.top, width, height: safeHeight };
    }

    const height = safeWidth / fieldAspect;
    const top = rect.top + (safeHeight - height) / 2;
    return { left: rect.left, top, width: safeWidth, height };
  }

  function trajectoryPath(): string {
    if (trajectory.length < 2) return '';
    return trajectory
      .map((point, index) => {
        const x = clamp(point.x, 0, config.fieldLength);
        const y = clamp(point.y, 0, config.fieldWidth);
        return `${index === 0 ? 'M' : 'L'} ${x.toFixed(3)} ${toFieldY(y).toFixed(3)}`;
      })
      .join(' ');
  }

  function setPoseAtClientPoint(clientX: number, clientY: number) {
    if (!config.allowPoseSet) return;
    if (config.xSignalId === null || config.ySignalId === null) return;

    if (!svgEl) return;
    const rect = svgEl.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return;
    const viewport = fieldViewport(rect);

    const normalizedX = clamp((clientX - viewport.left) / viewport.width, 0, 1);
    const normalizedY = clamp((clientY - viewport.top) / viewport.height, 0, 1);

    const x = normalizedX * config.fieldLength;
    const y = (1 - normalizedY) * config.fieldWidth;

    onSendSet(config.xSignalId, x.toFixed(3));
    onSendSet(config.ySignalId, y.toFixed(3));
  }

  function onFieldClick(event: MouseEvent) {
    setPoseAtClientPoint(event.clientX, event.clientY);
  }

  function onFieldKeydown(event: KeyboardEvent) {
    if (event.key !== 'Enter' && event.key !== ' ') return;
    event.preventDefault();
    if (!svgEl) return;
    const rect = svgEl.getBoundingClientRect();
    const viewport = fieldViewport(rect);
    setPoseAtClientPoint(viewport.left + viewport.width / 2, viewport.top + viewport.height / 2);
  }
</script>

<div class="field-root" data-interactive={config.allowPoseSet}>
  <svg
    bind:this={svgEl}
    viewBox={`0 0 ${config.fieldLength} ${config.fieldWidth}`}
    preserveAspectRatio="xMidYMid meet"
    role="button"
    tabindex={config.allowPoseSet ? 0 : -1}
    aria-label={config.allowPoseSet ? 'field viewer, click to set pose' : 'field viewer'}
    onclick={onFieldClick}
    onkeydown={onFieldKeydown}
  >
    <rect class="field-bg" x="0" y="0" width={config.fieldLength} height={config.fieldWidth} />
    {#if fieldImageUrl}
      <image
        x="0"
        y="0"
        width={config.fieldLength}
        height={config.fieldWidth}
        href={fieldImageUrl}
        preserveAspectRatio="xMidYMid meet"
        opacity={config.imageOpacity}
      />
    {/if}
    <line
      class="center-line"
      x1={(config.fieldLength / 2).toFixed(3)}
      y1="0"
      x2={(config.fieldLength / 2).toFixed(3)}
      y2={config.fieldWidth.toFixed(3)}
    />

    {#if trajectory.length > 1}
      <path class="trajectory" d={trajectoryPath()} />
    {/if}

    {#if pose.x !== null && pose.y !== null}
      {@const robotX = clamp(pose.x, 0, config.fieldLength)}
      {@const robotY = clamp(pose.y, 0, config.fieldWidth)}
      <g transform={`translate(${robotX.toFixed(3)} ${toFieldY(robotY).toFixed(3)}) rotate(${-pose.heading.toFixed(2)})`}>
        <rect class="robot-body" x="-0.32" y="-0.32" width="0.64" height="0.64" rx="0.06" />
        <line class="robot-forward" x1="0" y1="0" x2="0" y2="-0.52" />
        <polygon class="robot-forward-head" points="0,-0.64 -0.09,-0.46 0.09,-0.46" />
        <circle class="robot-hub" cx="0" cy="0" r="0.11" />
      </g>
    {/if}
  </svg>

  <div class="field-meta">
    <span>X: {pose.x === null ? 'n/a' : pose.x.toFixed(2)}</span>
    <span>Y: {pose.y === null ? 'n/a' : pose.y.toFixed(2)}</span>
    <span>H: {pose.heading.toFixed(1)} deg</span>
  </div>

  {#if config.allowPoseSet}
    <div class="field-hint">Click field to set pose</div>
  {/if}
</div>

<style>
  .field-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto auto;
    gap: 0.2rem;
  }

  svg {
    width: 100%;
    height: 100%;
    min-height: 4.2rem;
    border-radius: 7px;
    border: 1px solid var(--border-subtle);
    background: #121927;
    cursor: default;
  }

  .field-root[data-interactive='true'] svg {
    cursor: crosshair;
  }

  .field-bg {
    fill: #111827;
    stroke: rgba(148, 163, 184, 0.3);
    stroke-width: 0.04;
  }

  .center-line {
    stroke: rgba(148, 163, 184, 0.24);
    stroke-width: 0.05;
    stroke-dasharray: 0.25 0.2;
  }

  .trajectory {
    fill: none;
    stroke: rgba(56, 189, 248, 0.95);
    stroke-width: 0.08;
  }

  .robot-body {
    fill: rgba(248, 113, 113, 0.95);
    stroke: rgba(254, 226, 226, 0.8);
    stroke-width: 0.03;
  }

  .robot-forward {
    stroke: rgba(254, 226, 226, 0.95);
    stroke-width: 0.05;
    stroke-linecap: round;
  }

  .robot-forward-head {
    fill: rgba(254, 226, 226, 0.95);
  }

  .robot-hub {
    fill: rgba(17, 24, 39, 0.9);
    stroke: rgba(254, 226, 226, 0.8);
    stroke-width: 0.02;
  }

  .field-meta {
    display: flex;
    justify-content: space-between;
    gap: 0.2rem;
    color: var(--text-soft);
    font-size: 0.62rem;
    font-family: var(--font-mono);
  }

  .field-hint {
    color: var(--text-soft);
    font-size: 0.6rem;
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }
</style>
