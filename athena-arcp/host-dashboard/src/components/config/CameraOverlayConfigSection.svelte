<script lang="ts">
  import { readCameraOverlayConfig } from '../../lib/widget-config';
  import type { SignalSectionProps } from './types';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const stringSignals = $derived(pools.stringSignals);

</script>

{#if selectedWidget.kind === 'camera_overlay'}
  {@const camera = readCameraOverlayConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Camera + Overlay</h4>
    <div class="row-grid">
      <label>
        Stream signal
        <select
          value={camera.streamSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, streamSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Stream URL override
        <input
          value={camera.streamUrl}
          oninput={(event) => setConfig({ ...camera, streamUrl: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Pose X signal
        <select
          value={camera.poseXSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, poseXSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Pose Y signal
        <select
          value={camera.poseYSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, poseYSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Heading signal
        <select
          value={camera.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Targets signal
        <select
          value={camera.targetsSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, targetsSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Detections signal
        <select
          value={camera.detectionsSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...camera, detectionsSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Source width
        <input
          type="number"
          min="1"
          value={camera.sourceWidth}
          oninput={(event) =>
            setConfig({ ...camera, sourceWidth: Math.max(1, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 1280)) })}
        />
      </label>

      <label>
        Source height
        <input
          type="number"
          min="1"
          value={camera.sourceHeight}
          oninput={(event) =>
            setConfig({ ...camera, sourceHeight: Math.max(1, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 720)) })}
        />
      </label>
    </div>

    <div class="toggle-grid">
      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showPose}
          onchange={(event) => setConfig({ ...camera, showPose: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show pose
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showTargets}
          onchange={(event) => setConfig({ ...camera, showTargets: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show targets
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.showDetections}
          onchange={(event) => setConfig({ ...camera, showDetections: (event.currentTarget as HTMLInputElement).checked })}
        />
        Show detections
      </label>

      <label class="row-inline">
        <input
          type="checkbox"
          checked={camera.mirrorX}
          onchange={(event) => setConfig({ ...camera, mirrorX: (event.currentTarget as HTMLInputElement).checked })}
        />
        Mirror X
      </label>
    </div>
  </section>
{/if}
