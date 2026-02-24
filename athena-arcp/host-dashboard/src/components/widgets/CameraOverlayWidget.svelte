<script lang="ts">
  import { onMount } from 'svelte';
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseOverlayBoxes, readCameraOverlayConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  let { signal, signals, signalById, configRaw }: Props = $props();
  let streamWrapEl = $state<HTMLDivElement | null>(null);
  let streamViewport = $state({ left: 0, top: 0, width: 0, height: 0 });

  function updateViewportRect() {
    const host = streamWrapEl;
    if (!host) return;

    const hostWidth = host.clientWidth;
    const hostHeight = host.clientHeight;
    if (hostWidth <= 0 || hostHeight <= 0) return;

    const config = readCameraOverlayConfig(configRaw, signal, signals);
    const sourceWidth = Math.max(1, config.sourceWidth);
    const sourceHeight = Math.max(1, config.sourceHeight);
    const sourceAspect = sourceWidth / sourceHeight;
    const hostAspect = hostWidth / hostHeight;

    let width = hostWidth;
    let height = hostHeight;
    let left = 0;
    let top = 0;

    // Match object-fit: contain letterboxing so overlay coordinates map to the rendered image.
    if (hostAspect > sourceAspect) {
      height = hostHeight;
      width = height * sourceAspect;
      left = (hostWidth - width) / 2;
    } else {
      width = hostWidth;
      height = width / sourceAspect;
      top = (hostHeight - height) / 2;
    }

    streamViewport = { left, top, width, height };
  }

  onMount(() => {
    updateViewportRect();

    const observer = new ResizeObserver(() => {
      updateViewportRect();
    });
    if (streamWrapEl) {
      observer.observe(streamWrapEl);
    }

    window.addEventListener('resize', updateViewportRect);
    return () => {
      observer.disconnect();
      window.removeEventListener('resize', updateViewportRect);
    };
  });

  function numericSignalValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function resolvedStreamUrl(config: ReturnType<typeof readCameraOverlayConfig>): string {
    if (config.streamUrl.trim()) return config.streamUrl.trim();
    if (config.streamSignalId === null) return '';
    const row = signalById.get(config.streamSignalId);
    if (!row) return '';
    return row.value.trim();
  }

  const computed = $derived.by(() => {
    const config = readCameraOverlayConfig(configRaw, signal, signals);

    const streamUrl = resolvedStreamUrl(config);
    const poseX = numericSignalValue(config.poseXSignalId);
    const poseY = numericSignalValue(config.poseYSignalId);
    const heading = numericSignalValue(config.headingSignalId);

    const targetsRaw = config.targetsSignalId === null ? '' : signalById.get(config.targetsSignalId)?.value ?? '';
    const detectionsRaw =
      config.detectionsSignalId === null ? '' : signalById.get(config.detectionsSignalId)?.value ?? '';

    return {
      config,
      streamUrl,
      poseX,
      poseY,
      heading,
      targets: parseOverlayBoxes(targetsRaw, config.sourceWidth, config.sourceHeight),
      detections: parseOverlayBoxes(detectionsRaw, config.sourceWidth, config.sourceHeight)
    };
  });

  $effect(() => {
    // Recompute when source dimensions/config/signal stream context changes.
    computed.config.sourceWidth;
    computed.config.sourceHeight;
    computed.config.streamSignalId;
    computed.config.streamUrl;
    updateViewportRect();
  });
</script>

<div class="camera-root">
  <div class={`stream-wrap ${computed.config.mirrorX ? 'mirror' : ''}`} bind:this={streamWrapEl}>
    {#if computed.streamUrl}
      <img src={computed.streamUrl} alt="camera stream" loading="lazy" />
    {:else}
      <div class="no-stream">No stream source configured</div>
    {/if}

    <div class="overlay-layer" aria-hidden="true">
      <div
        class="overlay-viewport"
        style={`left:${streamViewport.left}px;top:${streamViewport.top}px;width:${streamViewport.width}px;height:${streamViewport.height}px;`}
      >
        {#if computed.config.showTargets}
          {#each computed.targets as box, index (`t-${index}`)}
            <div
              class="box target"
              style={`left:${(box.x * 100).toFixed(3)}%;top:${(box.y * 100).toFixed(3)}%;width:${(box.w * 100).toFixed(3)}%;height:${(box.h * 100).toFixed(3)}%;`}
            >
              {#if box.label || box.score !== null}
                <span>{box.label || 'target'}{box.score !== null ? ` ${(box.score * 100).toFixed(0)}%` : ''}</span>
              {/if}
            </div>
          {/each}
        {/if}

        {#if computed.config.showDetections}
          {#each computed.detections as box, index (`d-${index}`)}
            <div
              class="box detection"
              style={`left:${(box.x * 100).toFixed(3)}%;top:${(box.y * 100).toFixed(3)}%;width:${(box.w * 100).toFixed(3)}%;height:${(box.h * 100).toFixed(3)}%;`}
            >
              {#if box.label || box.score !== null}
                <span>{box.label || 'det'}{box.score !== null ? ` ${(box.score * 100).toFixed(0)}%` : ''}</span>
              {/if}
            </div>
          {/each}
        {/if}

        {#if computed.config.showPose}
          <div class="pose-hud">
            <strong>Pose</strong>
            <span>X {computed.poseX === null ? 'n/a' : computed.poseX.toFixed(2)}</span>
            <span>Y {computed.poseY === null ? 'n/a' : computed.poseY.toFixed(2)}</span>
            <span>H {computed.heading === null ? 'n/a' : `${computed.heading.toFixed(1)} deg`}</span>
          </div>
        {/if}
      </div>
    </div>
  </div>

  <div class="camera-meta">
    <span>{computed.targets.length} targets</span>
    <span>{computed.detections.length} detections</span>
    {#if computed.streamUrl}
      <span class="stream-text" title={computed.streamUrl}>{computed.streamUrl}</span>
    {/if}
  </div>
</div>

<style>
  .camera-root {
    height: 100%;
    min-width: 0;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.22rem;
  }

  .stream-wrap {
    position: relative;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    overflow: hidden;
    background: #0f172a;
    min-width: 0;
    min-height: 4rem;
  }

  .stream-wrap.mirror img {
    transform: scaleX(-1);
  }

  img {
    width: 100%;
    height: 100%;
    display: block;
    object-fit: contain;
    background: #0b1220;
  }

  .no-stream {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    color: var(--text-soft);
    font-size: 0.72rem;
    background: linear-gradient(180deg, rgba(15, 23, 42, 0.88), rgba(15, 23, 42, 0.96));
  }

  .overlay-layer {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }

  .overlay-viewport {
    position: absolute;
    overflow: hidden;
  }

  .stream-wrap.mirror .overlay-viewport {
    transform: scaleX(-1);
    transform-origin: center;
  }

  .box {
    position: absolute;
    border-radius: 4px;
    border: 2px solid;
    box-shadow: inset 0 0 0 1px rgba(15, 23, 42, 0.75);
  }

  .box span {
    position: absolute;
    left: 0;
    top: -1.15rem;
    max-width: 12rem;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    border: 1px solid;
    border-radius: 4px;
    padding: 0.07rem 0.24rem;
    font-size: 0.58rem;
    font-family: var(--font-mono);
    background: rgba(2, 6, 23, 0.84);
  }

  .stream-wrap.mirror .box span {
    transform: scaleX(-1);
    transform-origin: left top;
  }

  .box.target {
    border-color: rgba(56, 189, 248, 0.95);
  }

  .box.target span {
    border-color: rgba(56, 189, 248, 0.95);
    color: #bae6fd;
  }

  .box.detection {
    border-color: rgba(249, 115, 22, 0.95);
  }

  .box.detection span {
    border-color: rgba(249, 115, 22, 0.95);
    color: #fed7aa;
  }

  .pose-hud {
    position: absolute;
    left: 0.36rem;
    top: 0.36rem;
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    background: rgba(2, 6, 23, 0.78);
    color: var(--text);
    display: grid;
    gap: 0.05rem;
    padding: 0.22rem 0.32rem;
    font-size: 0.62rem;
    font-family: var(--font-mono);
    line-height: 1.15;
  }

  .pose-hud strong {
    color: var(--text-strong);
    font-family: var(--font-display);
    font-size: 0.66rem;
    line-height: 1.05;
  }

  .camera-meta {
    display: flex;
    align-items: center;
    gap: 0.3rem;
    flex-wrap: wrap;
    min-width: 0;
    overflow: hidden;
    color: var(--text-soft);
    font-size: 0.62rem;
  }

  .camera-meta span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.1rem 0.34rem;
    background: var(--surface-3);
    min-width: 0;
    max-width: 100%;
  }

  .camera-meta .stream-text {
    display: block;
    flex: 1 1 100%;
    width: 100%;
    max-width: 100%;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
    font-family: var(--font-mono);
  }
</style>
