<script lang="ts">
  import { readBarConfig, readDialConfig, readImu3dConfig, readTimerConfig } from '../../lib/widget-config';
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

</script>

{#if selectedWidget.kind === 'bar'}
  {@const bar = readBarConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Bar/Meter</h4>
    <div class="row-grid">
      <label>
        Min
        <input
          type="number"
          value={bar.min}
          oninput={(event) => setConfig({ ...bar, min: Number((event.currentTarget as HTMLInputElement).value) || 0 })}
        />
      </label>
      <label>
        Max
        <input
          type="number"
          value={bar.max}
          oninput={(event) => setConfig({ ...bar, max: Number((event.currentTarget as HTMLInputElement).value) || 1 })}
        />
      </label>
      <label>
        Unit
        <input
          value={bar.unit}
          oninput={(event) => setConfig({ ...bar, unit: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Mode
        <select
          value={bar.mode}
          onchange={(event) =>
            setConfig({ ...bar, mode: (event.currentTarget as HTMLSelectElement).value as 'progress' | 'usage' })}
        >
          <option value="progress">Progress</option>
          <option value="usage">Usage</option>
        </select>
      </label>
      <label>
        Warn (0-1)
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={bar.warn}
          oninput={(event) => setConfig({ ...bar, warn: Number((event.currentTarget as HTMLInputElement).value) || 0.7 })}
        />
      </label>
      <label>
        Crit (0-1)
        <input
          type="number"
          min="0"
          max="1"
          step="0.05"
          value={bar.crit}
          oninput={(event) => setConfig({ ...bar, crit: Number((event.currentTarget as HTMLInputElement).value) || 0.9 })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'dial'}
  {@const dial = readDialConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Dial</h4>
    <div class="row-grid">
      <label>
        Min
        <input
          type="number"
          value={dial.min}
          oninput={(event) => setConfig({ ...dial, min: Number((event.currentTarget as HTMLInputElement).value) || 0 })}
        />
      </label>
      <label>
        Max
        <input
          type="number"
          value={dial.max}
          oninput={(event) => setConfig({ ...dial, max: Number((event.currentTarget as HTMLInputElement).value) || 100 })}
        />
      </label>
      <label>
        Unit
        <input
          value={dial.unit}
          oninput={(event) => setConfig({ ...dial, unit: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'imu_3d'}
  {@const imu3d = readImu3dConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>3D IMU</h4>
    <div class="row-grid">
      <label>
        Roll signal
        <select
          value={imu3d.rollSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, rollSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Pitch signal
        <select
          value={imu3d.pitchSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, pitchSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Yaw signal
        <select
          value={imu3d.yawSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...imu3d, yawSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Packed/auto</option>
          {#each numericSignals as signalOption}
            <option value={signalOption.signal_id}>{signalOption.path}</option>
          {/each}
        </select>
      </label>
      <label>
        Units
        <select
          value={imu3d.units}
          onchange={(event) =>
            setConfig({
              ...imu3d,
              units: (event.currentTarget as HTMLSelectElement).value === 'rad' ? 'rad' : 'deg'
            })}
        >
          <option value="deg">Degrees</option>
          <option value="rad">Radians</option>
        </select>
      </label>
    </div>
    <p class="hint">Uses [roll, pitch, yaw] when signal value is a numeric array.</p>
  </section>
{:else if selectedWidget.kind === 'timer'}
  {@const timer = readTimerConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Timer</h4>
    <div class="row-grid">
      <label>
        Mode
        <select
          value={timer.mode}
          onchange={(event) =>
            setConfig({ ...timer, mode: (event.currentTarget as HTMLSelectElement).value as 'elapsed' | 'countdown' })}
        >
          <option value="countdown">Countdown</option>
          <option value="elapsed">Elapsed</option>
        </select>
      </label>
      <label>
        Duration (sec)
        <input
          type="number"
          min="1"
          value={timer.durationSec}
          oninput={(event) =>
            setConfig({ ...timer, durationSec: Math.max(1, Number((event.currentTarget as HTMLInputElement).value) || 150) })}
        />
      </label>
    </div>
  </section>
{/if}
