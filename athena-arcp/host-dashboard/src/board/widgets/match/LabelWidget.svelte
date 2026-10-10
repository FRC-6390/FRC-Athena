<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { unescapeLines } from '../../../lib/board/widgets/match-layout';

  let { ctx }: { ctx: WidgetContext } = $props();

  const text = $derived(unescapeLines(ctx.setting<string>('text')));
  const size = $derived(ctx.setting<string>('size'));
  const align = $derived(ctx.setting<string>('align'));
  const tone = $derived(ctx.setting<string>('tone'));
  const lines = $derived(text.split('\n'));
</script>

<div class="label" data-align={align} data-tone={tone} data-size={size}>
  <div class="text">
    {#each lines as line, i (i)}
      <span class="line">{line === '' ? ' ' : line}</span>
    {/each}
  </div>
</div>

<style>
  .label {
    width: 100%;
    height: 100%;
    display: flex;
    align-items: center;
    padding: 4px 12px;
    box-sizing: border-box;
    container-type: size;
    color: var(--text-strong);
  }
  .label[data-align='center'] {
    justify-content: center;
    text-align: center;
  }
  .label[data-align='right'] {
    justify-content: flex-end;
    text-align: right;
  }
  .text {
    font-weight: 700;
    line-height: 1.1;
    letter-spacing: -0.01em;
    min-width: 0;
    overflow: hidden;
    font-size: min(28px, 40cqh);
  }
  .label[data-size='small'] .text {
    font-size: min(16px, 24cqh);
  }
  .label[data-size='medium'] .text {
    font-size: min(28px, 40cqh);
  }
  .label[data-size='large'] .text {
    font-size: min(60cqh, 14cqw);
  }
  .label[data-size='huge'] .text {
    font-size: min(90cqh, 20cqw);
  }
  .label[data-size='auto'] .text {
    font-size: min(70cqh, 12cqw);
  }
  .label[data-tone='soft'] .text {
    color: var(--text-soft);
  }
  .label[data-tone='brand'] .text {
    color: var(--brand);
  }
  .label[data-tone='info'] .text {
    color: var(--info);
  }
  .label[data-tone='ok'] .text {
    color: var(--ok);
  }
  .label[data-tone='warn'] .text {
    color: var(--warn);
  }
  .line {
    display: block;
    white-space: pre-wrap;
    overflow-wrap: anywhere;
  }
</style>
