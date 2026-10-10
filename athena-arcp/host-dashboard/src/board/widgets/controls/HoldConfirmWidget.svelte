<script lang="ts">
  import { onDestroy } from 'svelte';
  import type { WidgetContext } from '../../../lib/board/types';
  import { finiteSetting } from '../../../lib/board/widgets/controls-numeric';

  let { ctx }: { ctx: WidgetContext } = $props();

  const action = $derived(ctx.bindings.action?.[0]);
  const holdMs = $derived(Math.max(200, finiteSetting(ctx.setting<number>('holdMs')) ?? 1500));
  const tone = $derived(ctx.setting<string>('tone') || 'warn');
  const running = $derived.by(() => {
    void ctx.live.current;
    return ctx.latest(action) === true;
  });
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = action ? ctx.reply(action) : undefined;
    return r && (r.op === 'run' || r.op === 'cancel') && !r.accepted ? r : undefined;
  });
  const label = $derived(ctx.setting<string>('label') || (action ? action.name.replace(/([a-z])([A-Z])/g, '$1 $2') : 'Action'));

  /** 0..1 fill while the button is held. */
  let progress = $state(0);
  let frame = 0;
  let startedAt = 0;
  let fired = false;
  let held = false;

  function tick(now: number) {
    progress = Math.min(1, (now - startedAt) / holdMs);
    if (progress >= 1) {
      fired = true;
      stop();
      if (action) void ctx.run(action);
      return;
    }
    frame = requestAnimationFrame(tick);
  }

  function stop() {
    cancelAnimationFrame(frame);
    frame = 0;
  }

  function begin(event?: Event) {
    if (!action || ctx.editing || held) return;
    if (event && 'button' in event && (event as PointerEvent).button !== 0) return;
    held = true;
    // A press while the action is running stops it right away, which is always the safe direction.
    if (running) {
      held = false;
      void ctx.cancel(action);
      return;
    }
    fired = false;
    startedAt = performance.now();
    progress = 0;
    stop();
    frame = requestAnimationFrame(tick);
  }

  function end() {
    if (!held) return;
    held = false;
    stop();
    if (!fired) progress = 0;
    fired = false;
  }

  onDestroy(stop);
</script>

<div class="hold" data-tone={tone} class:firing={progress >= 1}>
  <button
    type="button"
    class="hold-btn"
    aria-label={`Hold to run: ${label}`}
    disabled={!action || ctx.editing}
    onpointerdown={(e) => {
      e.currentTarget.setPointerCapture?.(e.pointerId);
      begin(e);
    }}
    onpointerup={end}
    onpointercancel={end}
    onlostpointercapture={end}
    onkeydown={(e) => {
      if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) begin();
    }}
    onkeyup={(e) => {
      if (e.key === ' ' || e.key === 'Enter') end();
    }}
    oncontextmenu={(e) => e.preventDefault()}
  >
    <span class="fill" style="transform: scaleX({progress})"></span>
    <span class="content">
      <span class="name">{label}</span>
      <span class="state">
        {#if running}Running · press to stop
        {:else if progress >= 1}Sent
        {:else if progress > 0}Keep holding…
        {:else}Hold {(holdMs / 1000).toFixed(holdMs % 1000 === 0 ? 0 : 1)} s to run{/if}
      </span>
    </span>
  </button>
  {#if reply}<span class="reason" title={reply.reason}>rejected{reply.reason ? `: ${reply.reason}` : ''}</span>{/if}
</div>

<style>
  .hold {
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 4px;
    height: 100%;
    padding: 6px;
    min-width: 0;
  }
  .hold-btn {
    position: relative;
    display: grid;
    place-items: center;
    overflow: hidden;
    min-height: 36px;
    border: 1px solid var(--border-emphasis);
    border-radius: var(--radius);
    background: var(--surface-3);
    color: var(--text-strong);
    font: inherit;
    cursor: pointer;
    user-select: none;
    touch-action: none;
  }
  .hold[data-tone='warn'] .hold-btn {
    border-color: var(--warn);
  }
  .hold[data-tone='fault'] .hold-btn {
    border-color: var(--fault);
  }
  .hold[data-tone='info'] .hold-btn {
    border-color: var(--info);
  }
  .hold-btn:disabled {
    cursor: default;
    opacity: 0.5;
  }
  .fill {
    position: absolute;
    inset: 0;
    transform-origin: left center;
    background: var(--warn-soft);
    pointer-events: none;
  }
  .hold[data-tone='fault'] .fill {
    background: var(--fault-soft);
  }
  .hold[data-tone='info'] .fill {
    background: var(--info-soft);
  }
  .hold.firing .fill {
    background: var(--ok-soft);
  }
  .content {
    position: relative;
    display: grid;
    gap: 2px;
    justify-items: center;
    padding: 4px 8px;
    min-width: 0;
  }
  .name {
    font-size: var(--fs-lg);
    font-weight: 600;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    max-width: 100%;
  }
  .state {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .reason {
    font-size: var(--fs-xs);
    color: var(--fault);
    text-align: center;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
</style>
