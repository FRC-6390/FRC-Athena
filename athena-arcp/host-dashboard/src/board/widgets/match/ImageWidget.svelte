<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { safeImageUrl } from '../../../lib/board/widgets/match-layout';

  let { ctx }: { ctx: WidgetContext } = $props();

  const src = $derived(safeImageUrl(ctx.setting<string>('url')));
  const fit = $derived(ctx.setting<string>('fit'));
  const alt = $derived(ctx.setting<string>('alt'));
  let failed = $state(false);
  $effect(() => {
    void src;
    failed = false;
  });
</script>

<div class="image">
  {#if src && !failed}
    <img {src} alt={alt} style="object-fit:{fit}" onerror={() => (failed = true)} />
  {:else if src && failed}
    <span class="message">Image did not load</span>
  {:else}
    <span class="message">{ctx.setting<string>('url').trim() === '' ? 'No image URL set' : 'Image URL not allowed (use http, https, or a data image)'}</span>
  {/if}
</div>

<style>
  .image {
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    overflow: hidden;
    background: var(--bg-1);
    container-type: size;
  }
  img {
    width: 100%;
    height: 100%;
    display: block;
  }
  .message {
    padding: 10px;
    font-size: var(--fs-sm);
    text-align: center;
    color: var(--text-faint);
  }
</style>
