<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import {
    axisValue,
    isJoystickPath,
    joystickPrefix,
    mapJoystick,
    type AxisId,
    type ButtonId
  } from '../../../lib/board/widgets/match-controller';

  let { ctx }: { ctx: WidgetContext } = $props();

  const port = $derived(ctx.setting<number>('port'));
  const bound = $derived(ctx.bindings.input ?? []);

  const map = $derived.by(() => {
    void ctx.live.current;
    const store = ctx.session.store;
    const paths = bound.length > 0 ? bound.map((s) => s.path) : store.allSignals().map((s) => s.path).filter((p) => p.startsWith(joystickPrefix(port)));
    return mapJoystick(paths.filter(isJoystickPath), port);
  });

  function button(id: ButtonId): boolean {
    void ctx.live.current;
    const path = map.buttons[id];
    if (!path) return false;
    const signal = ctx.session.store.descriptorByPath(path);
    return signal ? ctx.latest(signal) === true : false;
  }

  function axis(id: AxisId): number {
    void ctx.live.current;
    const path = map.axes[id];
    if (!path) return 0;
    const signal = ctx.session.store.descriptorByPath(path);
    return signal ? axisValue(ctx.latest(signal), id) : 0;
  }

  const mapped = $derived(Object.keys(map.buttons).length + Object.keys(map.axes).length);
  const pressed = (id: ButtonId) => button(id);

  // Stick dots are placed inside a unit circle; y is inverted on screen (up is positive).
  const stick = (id: 'leftX' | 'rightX', idY: 'leftY' | 'rightY') => ({ x: axis(id), y: -axis(idY) });
  const left = $derived(stick('leftX', 'leftY'));
  const right = $derived(stick('rightX', 'rightY'));
  const lt = $derived(axis('leftTrigger'));
  const rt = $derived(axis('rightTrigger'));
  const formatAxis = (v: number) => v.toFixed(2);
</script>

<div class="pad">
  {#if mapped === 0}
    <div class="empty">No signals on joystick port {port}. Bind input signals in Edit, or change the port setting.</div>
  {:else}
    <svg viewBox="0 0 200 130" role="img" aria-label="Gamepad {port}">
      <path class="body" d="M 52 26 Q 100 14 148 26 Q 184 32 190 66 Q 194 110 168 116 Q 150 120 138 102 Q 128 90 100 90 Q 72 90 62 102 Q 50 120 32 116 Q 6 110 10 66 Q 16 32 52 26 Z" />

      <!-- Triggers and bumpers -->
      <rect class="trig" x="46" y="8" width="26" height="8" rx="2" />
      <rect class="trig" x="128" y="8" width="26" height="8" rx="2" />
      <rect class="bump" class:on={pressed('leftBumper')} x="36" y="16" width="40" height="8" rx="3" />
      <rect class="bump" class:on={pressed('rightBumper')} x="124" y="16" width="40" height="8" rx="3" />
      <rect class="fill" x="46" y="8" width="26" height={8} rx="2" style="transform-origin:0 0; transform: scaleX({lt})" />
      <rect class="fill" x="128" y="8" width="26" height={8} rx="2" style="transform-origin:0 0; transform: scaleX({rt})" />

      <!-- D-pad -->
      <rect class="btn" class:on={pressed('dpadUp')} x="53" y="50" width="10" height="10" rx="2" />
      <rect class="btn" class:on={pressed('dpadDown')} x="53" y="70" width="10" height="10" rx="2" />
      <rect class="btn" class:on={pressed('dpadLeft')} x="43" y="60" width="10" height="10" rx="2" />
      <rect class="btn" class:on={pressed('dpadRight')} x="63" y="60" width="10" height="10" rx="2" />

      <!-- Face buttons -->
      <circle class="btn" class:on={pressed('y')} cx="150" cy="48" r="6" />
      <circle class="btn" class:on={pressed('b')} cx="164" cy="60" r="6" />
      <circle class="btn" class:on={pressed('a')} cx="150" cy="72" r="6" />
      <circle class="btn" class:on={pressed('x')} cx="136" cy="60" r="6" />

      <!-- Centre -->
      <rect class="btn small" class:on={pressed('back')} x="84" y="54" width="12" height="7" rx="2" />
      <rect class="btn small" class:on={pressed('start')} x="104" y="54" width="12" height="7" rx="2" />

      <!-- Sticks -->
      <circle class="stick" cx={76 + left.x * 8} cy={84 + left.y * 8} r="9" class:on={pressed('leftStick')} />
      <circle class="stick" cx={124 + right.x * 8} cy={84 + right.y * 8} r="9" class:on={pressed('rightStick')} />
    </svg>
    <div class="axes ui-mono">
      <span>LX {formatAxis(left.x)}</span>
      <span>LY {formatAxis(left.y)}</span>
      <span>RX {formatAxis(right.x)}</span>
      <span>RY {formatAxis(right.y)}</span>
      <span>LT {formatAxis(lt)}</span>
      <span>RT {formatAxis(rt)}</span>
    </div>
    {#if map.unmapped.length > 0}
      <div class="other">{map.unmapped.length} other {map.unmapped.length === 1 ? 'signal' : 'signals'} on this port</div>
    {/if}
  {/if}
</div>

<style>
  .pad {
    width: 100%;
    height: 100%;
    display: grid;
    grid-template-rows: 1fr auto auto;
    align-items: center;
    justify-items: center;
    gap: 4px;
    padding: 6px;
    box-sizing: border-box;
    container-type: size;
  }
  svg {
    width: 100%;
    height: 100%;
    min-height: 0;
    max-height: 100%;
  }
  .body {
    fill: var(--surface-3);
    stroke: var(--border-emphasis);
    stroke-width: 1.2;
  }
  .trig {
    fill: var(--surface-2);
    stroke: var(--border-emphasis);
    stroke-width: 0.8;
  }
  .fill {
    fill: var(--info);
  }
  .bump,
  .btn,
  .stick {
    fill: var(--surface-2);
    stroke: var(--border-emphasis);
    stroke-width: 0.8;
  }
  .btn.small {
    fill: var(--surface);
  }
  .bump.on,
  .btn.on,
  .stick.on {
    fill: var(--info);
    stroke: var(--info);
  }
  .stick {
    fill: var(--surface-hover);
  }
  .axes {
    display: flex;
    flex-wrap: wrap;
    justify-content: center;
    gap: 2px 10px;
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .empty,
  .other {
    font-size: var(--fs-sm);
    color: var(--text-faint);
    text-align: center;
  }
</style>
