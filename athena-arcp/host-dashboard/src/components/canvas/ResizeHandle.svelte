<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faUpRightAndDownLeftFromCenter } from '@fortawesome/free-solid-svg-icons';

  let { onpointerdown }: { onpointerdown: (event: PointerEvent) => void } = $props();
</script>

<button
  class="resize-handle"
  title="Resize panel"
  aria-label="Resize panel"
  {onpointerdown}
  onclick={(event) => event.stopPropagation()}
>
  <FontAwesomeIcon icon={faUpRightAndDownLeftFromCenter} class="resize-handle-icon" />
</button>

<style>
  .resize-handle {
    position: absolute;
    right: 0.36rem;
    bottom: 0.26rem;
    width: 1.1rem;
    height: 1.1rem;
    display: inline-grid;
    place-items: center;
    border: none;
    background: transparent;
    color: var(--text-soft);
    cursor: nwse-resize;
    padding: 0;
    user-select: none;
    touch-action: none;
    z-index: 6;
  }

  .resize-handle :global(.resize-handle-icon) {
    width: 0.54rem;
    height: 0.54rem;
    display: block;
  }
</style>
