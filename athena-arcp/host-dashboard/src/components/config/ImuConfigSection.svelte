<script lang="ts">
  import SignalMapField from '../SignalMapField.svelte';
  import { readImuConfig } from '../../lib/widget-config';
  import type { ImuWidgetConfig } from '../../lib/widget-config';
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
  const boolSignals = $derived(pools.boolSignals);
  const stringSignals = $derived(pools.stringSignals);
  const vectorSignals = $derived(pools.vectorSignals);

</script>

{#if selectedWidget.kind === 'imu'}
  {@const imu = readImuConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>IMU</h4>
    <p class="hint">Auto-maps orientation, accel/gyro/mag vectors, plus CAN metadata controls.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readImuConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="row-grid">
      <SignalMapField
        label="Roll signal"
        selectedSignalId={imu.rollSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, rollSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Pitch signal"
        selectedSignalId={imu.pitchSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, pitchSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Yaw signal"
        selectedSignalId={imu.yawSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, yawSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Heading signal"
        selectedSignalId={imu.headingSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, headingSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Connected signal"
        selectedSignalId={imu.connectedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, connectedSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Accel vector signal"
        selectedSignalId={imu.accelSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, accelSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Gyro vector signal"
        selectedSignalId={imu.gyroSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, gyroSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Mag vector signal"
        selectedSignalId={imu.magSignalId}
        candidates={vectorSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, magSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="CAN ID signal"
        selectedSignalId={imu.canIdSignalId}
        candidates={numericSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, canIdSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="CAN bus signal"
        selectedSignalId={imu.canbusSignalId}
        candidates={stringSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, canbusSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Type signal"
        selectedSignalId={imu.typeSignalId}
        candidates={stringSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, typeSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <SignalMapField
        label="Inverted signal"
        selectedSignalId={imu.invertedSignalId}
        candidates={boolSignals}
        {signalById}
        onChange={(signalId) => setConfig({ ...imu, invertedSignalId: signalId } satisfies ImuWidgetConfig)}
      />
      <label>
        Units
        <select
          value={imu.units}
          onchange={(event) =>
            setConfig({ ...imu, units: (event.currentTarget as HTMLSelectElement).value === 'rad' ? 'rad' : 'deg' } satisfies ImuWidgetConfig)}
        >
          <option value="deg">Degrees</option>
          <option value="rad">Radians</option>
        </select>
      </label>
      <label>
        Orientation view
        <select
          value={imu.orientationViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              orientationViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['orientationViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Accel view
        <select
          value={imu.accelViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              accelViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['accelViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Gyro view
        <select
          value={imu.gyroViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              gyroViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['gyroViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
      <label>
        Mag view
        <select
          value={imu.magViewMode}
          onchange={(event) =>
            setConfig({
              ...imu,
              magViewMode: (event.currentTarget as HTMLSelectElement).value as ImuWidgetConfig['magViewMode']
            } satisfies ImuWidgetConfig)}
        >
          <option value="auto">Auto</option>
          <option value="3d">3D</option>
          <option value="2d">2D</option>
          <option value="1d">1D</option>
        </select>
      </label>
    </div>
  </section>
{/if}
