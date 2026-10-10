<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';

  let { ctx }: { ctx: WidgetContext } = $props();

  const action = $derived(ctx.bindings.action?.[0]);
  const running = $derived.by(() => {
    void ctx.live.current;
    return ctx.latest(action) === true;
  });
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = action ? ctx.reply(action) : undefined;
    return r && (r.op === 'run' || r.op === 'cancel') ? r : undefined;
  });
  const label = $derived(
    ctx.setting<string>('label') ||
      (action?.name ?? 'Action').replace(/([a-z])([A-Z])/g, '$1 $2').replace(/^./, (c) => c.toUpperCase())
  );
  const mode = $derived(ctx.setting<string>('mode'));

  function press() {
    if (!action || ctx.editing) return;
    if (mode === 'toggle' && running) void ctx.cancel(action);
    else if (mode !== 'hold') void ctx.run(action);
  }
  function down() {
    if (action && !ctx.editing && mode === 'hold') void ctx.run(action);
  }
  function up() {
    if (action && !ctx.editing && mode === 'hold') void ctx.cancel(action);
  }
</script>

<div class="wrap">
  <button
    type="button"
    class="action"
    class:running
    disabled={ctx.editing || !action}
    onclick={press}
    onpointerdown={down}
    onpointerup={up}
    onpointerleave={up}
  >
    <span class="name">{label}</span>
    <span class="status">{running ? 'Running' : mode === 'hold' ? 'Hold to run' : mode === 'toggle' ? 'Tap to start' : 'Tap to run'}</span>
  </button>
  {#if reply && !reply.accepted}<span class="rejected" title={reply.reason}>{reply.reason}</span>{/if}
</div>

<style>
  .wrap {
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    height: 100%;
    padding: 6px;
    gap: 4px;
  }
  .action {
    display: grid;
    place-content: center;
    gap: 2px;
    border: 1px solid var(--border-emphasis);
    border-radius: var(--radius);
    background: var(--surface-3);
    color: var(--text-strong);
    font: inherit;
    cursor: pointer;
    user-select: none;
    touch-action: none;
  }
  .action:hover:not(:disabled) {
    background: var(--surface-hover);
  }
  .action:active:not(:disabled) {
    transform: translateY(1px);
  }
  .action.running {
    border-color: var(--ok);
    background: var(--ok-soft);
  }
  .action:disabled {
    cursor: default;
  }
  .name {
    font-size: var(--fs-lg);
    font-weight: 600;
  }
  .status {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .running .status {
    color: var(--ok);
  }
  .rejected {
    font-size: var(--fs-xs);
    color: var(--fault);
    text-align: center;
  }
</style>
