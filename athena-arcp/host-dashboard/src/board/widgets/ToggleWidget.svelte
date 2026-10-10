<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const on = $derived.by(() => {
    void ctx.live.current;
    return ctx.latest(signal) === true;
  });
  const reply = $derived.by(() => {
    void ctx.live.current;
    const r = signal ? ctx.reply(signal) : undefined;
    return r && r.op === 'write' ? r : undefined;
  });
  const locked = $derived.by(() => {
    void ctx.live.current;
    return signal ? ctx.locked(signal) : false;
  });
</script>

<div class="wrap">
  <button
    type="button"
    class="switch"
    class:on
    role="switch"
    aria-checked={on}
    disabled={ctx.editing || !signal}
    onclick={() => signal && ctx.write(signal, !on)}
  >
    <span class="knob"></span>
  </button>
  <span class="state">{on ? ctx.setting<string>('onText') : ctx.setting<string>('offText')}</span>
  {#if locked}<span class="hint locked">locked</span>{/if}
  {#if reply && !reply.accepted}<span class="hint rejected" title={reply.reason}>{reply.reason}</span>{/if}
</div>

<style>
  .wrap {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 10px;
    height: 100%;
    flex-wrap: wrap;
    padding: 4px;
  }
  .switch {
    position: relative;
    width: 46px;
    height: 26px;
    border: 1px solid var(--border-emphasis);
    border-radius: 13px;
    background: var(--surface-3);
    cursor: pointer;
    transition: background 120ms;
    padding: 0;
  }
  .switch.on {
    background: var(--ok);
    border-color: var(--ok);
  }
  .switch:disabled {
    cursor: default;
  }
  .knob {
    position: absolute;
    top: 2px;
    left: 2px;
    width: 20px;
    height: 20px;
    border-radius: 50%;
    background: #fff;
    transition: transform 120ms;
  }
  .switch.on .knob {
    transform: translateX(20px);
  }
  .state {
    font-size: var(--fs-md);
    font-weight: 600;
    color: var(--text-strong);
  }
  .hint {
    font-size: var(--fs-xs);
  }
  .locked {
    color: var(--locked);
  }
  .rejected {
    color: var(--fault);
  }
</style>
