<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readImu3dConfig } from '../../lib/widget-config';
  import ImuOrientation3dScene from './ImuOrientation3dScene.svelte';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
  };

  type EulerAngles = {
    roll: number;
    pitch: number;
    yaw: number;
  };

  let { signal, signals, signalById, configRaw }: Props = $props();

  const config = $derived(readImu3dConfig(configRaw, signal, signals));
  const packed = $derived(parseNumericArray(signal.value));

  function numericSignalValue(signalId: number | null): number | null {
    if (signalId === null) return null;
    const row = signalById.get(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function isRollPath(path: string): boolean {
    const token = path.toLowerCase();
    return token.includes('roll') || token.endsWith('/phi');
  }

  function isPitchPath(path: string): boolean {
    const token = path.toLowerCase();
    return token.includes('pitch') || token.endsWith('/theta');
  }

  function isYawPath(path: string): boolean {
    const token = path.toLowerCase();
    return (
      token.includes('yaw') ||
      token.includes('heading') ||
      token.endsWith('/psi') ||
      token.includes('bearing')
    );
  }

  function radToDeg(value: number): number {
    return (value * 180) / Math.PI;
  }

  function wrapDeg(value: number): number {
    const wrapped = ((value + 180) % 360 + 360) % 360 - 180;
    return Number.isFinite(wrapped) ? wrapped : 0;
  }

  function quaternionToEulerDeg(w: number, x: number, y: number, z: number): EulerAngles {
    const sinr = 2 * (w * x + y * z);
    const cosr = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr, cosr);

    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp);

    const siny = 2 * (w * z + x * y);
    const cosy = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny, cosy);

    return {
      roll: radToDeg(roll),
      pitch: radToDeg(pitch),
      yaw: radToDeg(yaw)
    };
  }

  function packedEuler(): EulerAngles | null {
    if (packed.length >= 3) {
      return {
        roll: packed[0] ?? 0,
        pitch: packed[1] ?? 0,
        yaw: packed[2] ?? 0
      };
    }

    const lowerPath = signal.path.toLowerCase();
    if (packed.length >= 4 && (lowerPath.includes('quat') || lowerPath.includes('quaternion'))) {
      return quaternionToEulerDeg(packed[0] ?? 1, packed[1] ?? 0, packed[2] ?? 0, packed[3] ?? 0);
    }

    return null;
  }

  const angles = $derived.by(() => {
    const explicitRoll = numericSignalValue(config.rollSignalId);
    const explicitPitch = numericSignalValue(config.pitchSignalId);
    const explicitYaw = numericSignalValue(config.yawSignalId);
    const fromPacked = packedEuler();

    let roll =
      explicitRoll ??
      ((signal.signal_type === 'f64' || signal.signal_type === 'i64') && isRollPath(signal.path)
        ? Number(signal.value)
        : null) ??
      fromPacked?.roll ??
      0;

    let pitch =
      explicitPitch ??
      ((signal.signal_type === 'f64' || signal.signal_type === 'i64') && isPitchPath(signal.path)
        ? Number(signal.value)
        : null) ??
      fromPacked?.pitch ??
      0;

    let yaw =
      explicitYaw ??
      ((signal.signal_type === 'f64' || signal.signal_type === 'i64') && isYawPath(signal.path)
        ? Number(signal.value)
        : null) ??
      fromPacked?.yaw ??
      0;

    if (config.units === 'rad') {
      roll = radToDeg(roll);
      pitch = radToDeg(pitch);
      yaw = radToDeg(yaw);
    }

    return {
      roll: wrapDeg(roll),
      pitch: wrapDeg(pitch),
      yaw: wrapDeg(yaw)
    };
  });
</script>

<div class="imu-root">
  <div class="viewer">
    <ImuOrientation3dScene rollDeg={angles.roll} pitchDeg={angles.pitch} yawDeg={angles.yaw} />
  </div>

  <div class="meta">
    <span>R {angles.roll.toFixed(1)} deg</span>
    <span>P {angles.pitch.toFixed(1)} deg</span>
    <span>Y {angles.yaw.toFixed(1)} deg</span>
  </div>
</div>

<style>
  .imu-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.2rem;
  }

  .viewer {
    min-height: 0;
  }

  .meta {
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 0.42rem;
    font-size: 0.63rem;
    color: var(--text-soft);
    font-family: var(--font-mono);
    text-transform: uppercase;
    letter-spacing: 0.03em;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: rgba(15, 23, 42, 0.72);
    padding: 0.22rem 0.38rem;
  }

  .meta span {
    white-space: nowrap;
  }
</style>
