<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { parseInline, parseNote } from '../../../lib/board/widgets/match-layout';

  let { ctx }: { ctx: WidgetContext } = $props();

  const blocks = $derived(parseNote(ctx.setting<string>('text')));
  const dense = $derived(ctx.setting<string>('size') === 'small');
</script>

<div class="note" data-dense={dense}>
  {#each blocks as block, i (i)}
    {#if block.kind === 'heading'}
      <svelte:element this={block.level === 1 ? 'h2' : block.level === 2 ? 'h3' : 'h4'} class="h{block.level}">
        {#each parseInline(block.text) as seg, j (j)}{#if seg.bold}<strong>{seg.text}</strong>{:else}{seg.text}{/if}{/each}
      </svelte:element>
    {:else if block.kind === 'paragraph'}
      <p>
        {#each parseInline(block.text) as seg, j (j)}{#if seg.bold}<strong>{seg.text}</strong>{:else}{seg.text}{/if}{/each}
      </p>
    {:else}
      <ul>
        {#each block.items as item, j (j)}
          <li>{#each parseInline(item) as seg, k (k)}{#if seg.bold}<strong>{seg.text}</strong>{:else}{seg.text}{/if}{/each}</li>
        {/each}
      </ul>
    {/if}
  {:else}
    <p class="empty">Empty note. Add text in settings; use \n for line breaks.</p>
  {/each}
</div>

<style>
  .note {
    height: 100%;
    overflow: auto;
    padding: 8px 12px;
    box-sizing: border-box;
    font-size: var(--fs-lg);
    line-height: 1.45;
    color: var(--text);
    container-type: inline-size;
  }
  .note[data-dense='true'] {
    font-size: var(--fs-md);
  }
  h2,
  h3,
  h4,
  p,
  ul {
    margin: 0 0 0.5em;
  }
  h2 {
    font-size: 1.5em;
    line-height: 1.2;
    color: var(--text-strong);
  }
  h3 {
    font-size: 1.25em;
    color: var(--text-strong);
  }
  h4 {
    font-size: 1.1em;
    color: var(--text-strong);
  }
  ul {
    padding-left: 1.2em;
  }
  li {
    margin: 0.2em 0;
  }
  strong {
    color: var(--text-strong);
    font-weight: 700;
  }
  .empty {
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
