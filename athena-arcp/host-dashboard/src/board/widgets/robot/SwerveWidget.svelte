<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { DEG_TO_RAD, round, toDegrees, type AngleUnit } from '../../../lib/board/widgets/robot-geometry';
  import { numericOf } from '../../../lib/board/widgets/robot-series';
  import {
    MODULE_NAMES,
    chassisVelocity,
    modulePosition,
    moduleVelocity,
    parseModuleStates,
    type ModuleName,
    type ModuleSample,
    type ModuleState
  } from '../../../lib/board/widgets/robot-swerve';
  import { formatNumber } from '../../../lib/format';

  let { ctx }: { ctx: WidgetContext } = $props();

  const statesSignal = $derived(ctx.bindings.states?.[0]);
  const headingSignal = $derived(ctx.bindings.heading?.[0]);
  const unit = $derived(ctx.setting<string>('angleUnit') as AngleUnit);
  const angleScale = $derived(unit === 'deg' ? DEG_TO_RAD : 1);
  const wheelbase = $derived(Math.max(0.1, Number(ctx.setting<number>('wheelbase')) || 0.6));
  const trackwidth = $derived(Math.max(0.1, Number(ctx.setting<number>('trackwidth')) || 0.5));
  const maxSpeed = $derived(Math.max(0.1, Number(ctx.setting<number>('maxSpeed')) || 4));

  const slotKey = (name: ModuleName, what: 'speed' | 'angle') => `${name.toLowerCase()}_${what}`;

  /**
   * Per-module state: individual speed and angle slots win; otherwise a bound f64[] laid out as
   * [speed, angle] pairs in FL, FR, BL, BR order. Unbound modules read as stopped, facing forward.
   */
  const modules = $derived.by((): ModuleState[] => {
    void ctx.live.current;
    const flat = ctx.latest(statesSignal);
    const fromArray = Array.isArray(flat) && flat.length >= 8 ? parseModuleStates(flat as number[], angleScale) : null;
    return MODULE_NAMES.map((name, i) => {
      const speed = numericOf(ctx.latest(ctx.bindings[slotKey(name, 'speed')]?.[0]));
      const angle = numericOf(ctx.latest(ctx.bindings[slotKey(name, 'angle')]?.[0]));
      if (speed !== null || angle !== null) return { speed: speed ?? 0, angleRad: (angle ?? 0) * angleScale };
      return fromArray?.[i] ?? { speed: 0, angleRad: 0 };
    });
  });

  const samples = $derived<ModuleSample[]>(
    MODULE_NAMES.map((name, i) => {
      const p = modulePosition(name, wheelbase, trackwidth);
      const v = moduleVelocity(modules[i]);
      return { name, x: p.x, y: p.y, vx: v.x, vy: v.y };
    })
  );
  const chassis = $derived(chassisVelocity(samples));
  const headingDeg = $derived.by(() => {
    void ctx.live.current;
    const v = numericOf(ctx.latest(headingSignal));
    return v === null ? null : toDegrees(v, unit);
  });
  const hasData = $derived(
    headingDeg !== null || modules.some((m) => m.speed !== 0 || m.angleRad !== 0) || statesSignal !== undefined
  );

  // Geometry: the robot is drawn in its own frame (forward is up) and rotated by heading inside a fixed field view.
  const W = $derived(ctx.size.width);
  const H = $derived(ctx.size.height);
  const WHEEL_LEN = 0.14;
  const ARROW_M = 0.9;
  const halfL = $derived(wheelbase / 2);
  const halfW = $derived(trackwidth / 2);
  const k = $derived.by(() => {
    const extent = Math.max(halfL, halfW) + WHEEL_LEN / 2 + ARROW_M;
    return Math.max(4, (Math.min(W, H) / 2 - 16) / extent);
  });
  const cx = $derived(W / 2);
  const cy = $derived(H / 2 - 4);
  // Robot-frame metres to screen pixels: x forward maps to up, y left maps to left.
  const toScreen = (x: number, y: number) => ({ x: cx - y * k, y: cy - x * k });
  const fontPx = $derived(Math.max(9, Math.min(11, k * 0.14)));

  /** Rotates a point about the robot centre by `deg` (SVG convention, positive is clockwise on screen). */
  function rotatePoint(p: { x: number; y: number }, deg: number) {
    const r = (deg * Math.PI) / 180;
    const dx = p.x - cx;
    const dy = p.y - cy;
    return { x: cx + dx * Math.cos(r) - dy * Math.sin(r), y: cy + dx * Math.sin(r) + dy * Math.cos(r) };
  }
  const rotateDeg = () => (headingDeg === null ? 0 : -headingDeg);

  const wheels = $derived(
    MODULE_NAMES.map((name, i) => {
      const p = toScreen(modulePosition(name, wheelbase, trackwidth).x, modulePosition(name, wheelbase, trackwidth).y);
      const a = modules[i].angleRad;
      // Screen direction of the module's rolling axis (robot angle a, forward is screen up).
      const dx = -Math.sin(a);
      const dy = -Math.cos(a);
      const speed = modules[i].speed;
      const len = Math.min(1.6, Math.abs(speed) / maxSpeed) * ARROW_M * k;
      const sign = speed < 0 ? -1 : 1;
      const tip = { x: p.x + dx * len * sign, y: p.y + dy * len * sign };
      // Label sits outside the wheel, pushed away from the centre, then turned with the robot.
      const outX = p.x < cx ? -1 : 1;
      const outY = p.y < cy ? -1 : 1;
      const label = rotatePoint({ x: p.x + outX * 16, y: p.y + outY * 12 }, rotateDeg());
      return {
        name,
        p,
        tip,
        rotation: (Math.atan2(dy, dx) * 180) / Math.PI,
        speed,
        label,
        reversed: speed < 0,
        hasArrow: len > 0.5
      };
    })
  );
  const bodyW = $derived(trackwidth * k);
  const bodyH = $derived(wheelbase * k);
  const wheelW = $derived(WHEEL_LEN * k);
  const wheelT = $derived(Math.max(4, 0.07 * k));
  const rotate = $derived(rotateDeg());
</script>

<div class="swerve">
  <svg width={W} height={H} viewBox="0 0 {W} {H}" role="img" aria-label="swerve drive">
    <text x="8" y="14" class="field" font-size={fontPx}>field +x ↑</text>
    <g transform="rotate({rotate} {cx} {cy})">
      <rect x={cx - bodyW / 2} y={cy - bodyH / 2} width={bodyW} height={bodyH} rx="4" class="body" />
      <polygon points="{cx},{cy - bodyH / 2 - 6} {cx - 6},{cy - bodyH / 2 + 2} {cx + 6},{cy - bodyH / 2 + 2}" class="front" />
      {#each wheels as wheel (wheel.name)}
        <g transform="translate({round(wheel.p.x)} {round(wheel.p.y)}) rotate({round(wheel.rotation)})">
          <rect x={-wheelW / 2} y={-wheelT / 2} width={wheelW} height={wheelT} rx="2" class="wheel" class:reversed={wheel.reversed} />
        </g>
        {#if wheel.hasArrow}
          <line x1={round(wheel.p.x)} y1={round(wheel.p.y)} x2={round(wheel.tip.x)} y2={round(wheel.tip.y)} class="vector" />
          <circle cx={round(wheel.tip.x)} cy={round(wheel.tip.y)} r="2.5" class="vector-tip" />
        {/if}
      {/each}
    </g>
    {#each wheels as wheel (wheel.name)}
      <text x={round(wheel.label.x)} y={round(wheel.label.y)} class="module" font-size={fontPx} text-anchor="middle" dominant-baseline="central">
        {wheel.name} {formatNumber(wheel.speed)}
      </text>
    {/each}
  </svg>
  <div class="readout ui-mono">
    <span>vx {formatNumber(round(chassis.vx))}</span>
    <span>vy {formatNumber(round(chassis.vy))}</span>
    <span>ω {formatNumber(round(chassis.omega))}</span>
    <span class="unit">m/s, rad/s</span>
    {#if headingDeg !== null}<span class="heading">{formatNumber(round(headingDeg))}°</span>{/if}
    {#if !hasData}<span class="unit">bind states or module slots</span>{/if}
  </div>
</div>

<style>
  .swerve {
    position: relative;
    width: 100%;
    height: 100%;
    overflow: hidden;
  }
  svg {
    display: block;
  }
  .field {
    fill: var(--text-faint);
  }
  .body {
    fill: var(--surface-3);
    stroke: var(--border-emphasis);
    stroke-width: 1.5;
  }
  .front {
    fill: var(--info);
  }
  .wheel {
    fill: var(--text-soft);
  }
  .wheel.reversed {
    fill: var(--warn);
  }
  .vector {
    stroke: var(--info);
    stroke-width: 2.5;
    stroke-linecap: round;
  }
  .vector-tip {
    fill: var(--info);
  }
  .module {
    fill: var(--text-soft);
    font-family: var(--font-mono);
  }
  .readout {
    position: absolute;
    left: 8px;
    right: 8px;
    bottom: 4px;
    display: flex;
    flex-wrap: wrap;
    gap: 4px 12px;
    font-size: var(--fs-xs);
    color: var(--text);
    pointer-events: none;
  }
  .readout .unit {
    color: var(--text-faint);
  }
  .readout .heading {
    margin-left: auto;
    color: var(--text-strong);
    font-weight: 600;
  }
</style>
