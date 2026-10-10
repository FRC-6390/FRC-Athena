<script lang="ts">
  import type { WidgetContext } from '../../lib/board/types';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const on = $derived.by(() => {
    void ctx.live.current;
    return ctx.latest(signal) === true;
  });
  const trueTone = $derived(ctx.setting<string>('trueTone'));
  const falseTone = $derived(ctx.setting<string>('falseTone'));
  const trueText = $derived(ctx.setting<string>('trueText'));
  const falseText = $derived(ctx.setting<string>('falseText'));
</script>

<div class="light" data-tone={on ? trueTone : falseTone}>
  <span class="label">{on ? trueText : falseText}</span>
</div>

<style>
  .light {
    display: grid;
    place-items: center;
    height: 100%;
    margin: 6px;
    border-radius: var(--radius);
    background: var(--surface-3);
    transition: background 120ms;
  }
  .light[data-tone='ok'] { background: var(--ok); }
  .light[data-tone='warn'] { background: var(--warn); }
  .light[data-tone='fault'] { background: var(--fault); }
  .light[data-tone='info'] { background: var(--info); }
  .light[data-tone='off'] { background: var(--surface-3); }
  .label {
    font-size: var(--fs-md);
    font-weight: 700;
    letter-spacing: 0.04em;
    color: #fff;
    text-transform: uppercase;
  }
  .light[data-tone='off'] .label {
    color: var(--text-faint);
  }
</style>
