<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readImu3dConfig } from '../../lib/widget-config';

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

  const cubeTransform = $derived(
    `transform: rotateZ(${angles.yaw.toFixed(2)}deg) rotateX(${angles.pitch.toFixed(2)}deg) rotateY(${angles.roll.toFixed(2)}deg);`
  );
</script>

<div class="imu-root">
  <div class="viewer">
    <div class="scene">
      <div class="axes" aria-hidden="true">
        <span class="axis axis-x">X</span>
        <span class="axis axis-y">Y</span>
        <span class="axis axis-z">Z</span>
      </div>

      <div class="cube" style={cubeTransform} aria-label="3d imu orientation">
        <div class="face front">F</div>
        <div class="face back">B</div>
        <div class="face right">R</div>
        <div class="face left">L</div>
        <div class="face top">U</div>
        <div class="face bottom">D</div>
      </div>
    </div>
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
    border-radius: 7px;
    border: 1px solid var(--border-subtle);
    background: radial-gradient(circle at 50% 35%, #1a2436, #101827 70%);
    overflow: hidden;
    display: grid;
    place-items: center;
    padding: 0.2rem;
  }

  .scene {
    --cube-size: clamp(2.2rem, 52%, 6.3rem);
    width: 100%;
    height: 100%;
    min-height: 2.7rem;
    display: grid;
    place-items: center;
    perspective: 620px;
    position: relative;
  }

  .axes {
    position: absolute;
    inset: 0;
    pointer-events: none;
  }

  .axis {
    position: absolute;
    font-size: 0.54rem;
    font-family: var(--font-mono);
    color: rgba(226, 232, 240, 0.75);
    letter-spacing: 0.04em;
  }

  .axis-x {
    right: 0.34rem;
    top: 50%;
    transform: translateY(-50%);
    color: rgba(248, 113, 113, 0.9);
  }

  .axis-y {
    left: 50%;
    top: 0.26rem;
    transform: translateX(-50%);
    color: rgba(56, 189, 248, 0.9);
  }

  .axis-z {
    left: 0.34rem;
    bottom: 0.3rem;
    color: rgba(52, 211, 153, 0.9);
  }

  .cube {
    width: var(--cube-size);
    aspect-ratio: 1 / 1;
    position: relative;
    transform-style: preserve-3d;
    transition: transform 110ms linear;
  }

  .face {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    border-radius: 3px;
    border: 1px solid rgba(148, 163, 184, 0.46);
    background: rgba(15, 23, 42, 0.78);
    color: rgba(226, 232, 240, 0.88);
    font-size: 0.56rem;
    font-family: var(--font-mono);
    font-weight: 700;
  }

  .front {
    transform: translateZ(calc(var(--cube-size) / 2));
    border-color: rgba(248, 113, 113, 0.55);
  }

  .back {
    transform: rotateY(180deg) translateZ(calc(var(--cube-size) / 2));
  }

  .right {
    transform: rotateY(90deg) translateZ(calc(var(--cube-size) / 2));
    border-color: rgba(251, 191, 36, 0.5);
  }

  .left {
    transform: rotateY(-90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .top {
    transform: rotateX(90deg) translateZ(calc(var(--cube-size) / 2));
    border-color: rgba(56, 189, 248, 0.55);
  }

  .bottom {
    transform: rotateX(-90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .meta {
    display: flex;
    justify-content: space-between;
    gap: 0.3rem;
    color: var(--text-soft);
    font-size: 0.62rem;
    font-family: var(--font-mono);
  }
</style>
