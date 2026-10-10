<script lang="ts">
  import { readFieldConfig, readMech2dConfig } from '../../lib/widget-config';
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

{#if selectedWidget.kind === 'field'}
  {@const field = readFieldConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Field Viewer</h4>
    <div class="row-grid">
      <label>
        X signal
        <select
          value={field.xSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, xSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Y signal
        <select
          value={field.ySignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, ySignalId: raw ? Number(raw) : null });
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
          value={field.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Trajectory signal
        <select
          value={field.trajectorySignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, trajectorySignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Field image signal
        <select
          value={field.imageSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...field, imageSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each stringSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Field image URL override
        <input
          value={field.imageUrl}
          oninput={(event) => setConfig({ ...field, imageUrl: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Image opacity
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={field.imageOpacity}
          oninput={(event) =>
            setConfig({ ...field, imageOpacity: Math.max(0, Math.min(1, Number((event.currentTarget as HTMLInputElement).value) || 0)) })}
        />
      </label>

      <label>
        Field length (m)
        <input
          type="number"
          min="1"
          step="0.01"
          value={field.fieldLength}
          oninput={(event) =>
            setConfig({ ...field, fieldLength: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 16.54) })}
        />
      </label>

      <label>
        Field width (m)
        <input
          type="number"
          min="1"
          step="0.01"
          value={field.fieldWidth}
          oninput={(event) =>
            setConfig({ ...field, fieldWidth: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 8.02) })}
        />
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={field.allowPoseSet}
        onchange={(event) => setConfig({ ...field, allowPoseSet: (event.currentTarget as HTMLInputElement).checked })}
      />
      Allow click-to-set pose
    </label>
  </section>
{:else if selectedWidget.kind === 'mech2d'}
  {@const mech2d = readMech2dConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Mechanism2d</h4>
    <div class="row-grid">
      <label>
        Line width
        <input
          type="number"
          min="0.002"
          max="0.25"
          step="0.002"
          value={mech2d.lineWidth}
          oninput={(event) =>
            setConfig({
              ...mech2d,
              lineWidth: Math.max(0.002, Math.min(0.25, Number((event.currentTarget as HTMLInputElement).value) || mech2d.lineWidth))
            })}
        />
      </label>
    </div>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={mech2d.showGrid}
        onchange={(event) => setConfig({ ...mech2d, showGrid: (event.currentTarget as HTMLInputElement).checked })}
      />
      Show grid
    </label>
  </section>
{/if}
