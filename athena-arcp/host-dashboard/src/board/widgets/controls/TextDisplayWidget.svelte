<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { finiteSetting } from '../../../lib/board/widgets/controls-numeric';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.value?.[0]);
  const text = $derived.by(() => {
    void ctx.live.current;
    const v = ctx.latest(signal);
    if (typeof v === 'string') return v;
    if (v === undefined) return '--';
    return String(v);
  });
  const mono = $derived(ctx.setting<boolean>('mono') === true);
  const maxLines = $derived(Math.max(0, Math.floor(finiteSetting(ctx.setting<number>('maxLines')) ?? 0)));
  // Long text shrinks to fit the tile; short text stays readable.
  const clampCss = $derived(
    maxLines > 0 ? `display:-webkit-box;-webkit-box-orient:vertical;-webkit-line-clamp:${maxLines};line-clamp:${maxLines};overflow:hidden;` : ''
  );
  const fontSize = $derived(Math.max(11, Math.min(22, ctx.size.height * 0.3, ctx.size.width / Math.max(8, Math.min(text.length, 24) * 0.55))));
</script>

<div class="text" class:mono title={text}>
  <span class="body" style="font-size:{fontSize}px;{clampCss}">{text}</span>
</div>

<style>
  .text {
    height: 100%;
    padding: 8px 10px;
    overflow: auto;
    min-width: 0;
  }
  .body {
    display: block;
    color: var(--text-strong);
    white-space: pre-wrap;
    overflow-wrap: anywhere;
    line-height: 1.35;
  }
  .text.mono .body {
    font-family: var(--font-mono);
    font-variant-numeric: tabular-nums;
  }
</style>
