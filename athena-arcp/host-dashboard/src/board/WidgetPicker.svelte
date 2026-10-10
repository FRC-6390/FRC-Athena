<script lang="ts">
  import { onMount } from 'svelte';
  import type { SignalDescriptor } from '../lib/data/types';
  import type { WidgetDef } from '../lib/board/types';

  type Props = {
    signal: SignalDescriptor;
    options: WidgetDef[];
    /** Viewport position of the drop. */
    at: { left: number; top: number };
    onpick: (def: WidgetDef) => void;
    oncancel: () => void;
  };

  let { signal, options, at, onpick, oncancel }: Props = $props();
  let active = $state(0);
  let menu = $state<HTMLDivElement | null>(null);

  onMount(() => menu?.focus());

  function key(event: KeyboardEvent) {
    if (event.key === 'ArrowDown') active = (active + 1) % options.length;
    else if (event.key === 'ArrowUp') active = (active - 1 + options.length) % options.length;
    else if (event.key === 'Enter' && options[active]) onpick(options[active]);
    else if (event.key === 'Escape') oncancel();
    else return;
    event.preventDefault();
  }
</script>

<div class="scrim" role="presentation" onpointerdown={oncancel}></div>
<div
  class="picker ui-panel"
  role="listbox"
  tabindex="-1"
  aria-label="Choose a widget"
  bind:this={menu}
  onkeydown={key}
  style="left:{Math.min(at.left, window.innerWidth - 300)}px; top:{Math.min(at.top, window.innerHeight - 320)}px"
>
  <div class="head">
    <span class="ui-label">Show</span>
    <span class="path ui-mono">{signal.path}</span>
  </div>
  {#each options as def, i (def.type)}
    <button
      type="button"
      role="option"
      aria-selected={i === active}
      class:active={i === active}
      onpointerenter={() => (active = i)}
      onclick={() => onpick(def)}
    >
      <span class="name">{def.name}</span>
      <span class="desc">{def.description}</span>
    </button>
  {:else}
    <p class="none">No widget can show a {signal.type} signal yet.</p>
  {/each}
</div>

<style>
  .scrim {
    position: fixed;
    inset: 0;
    z-index: 40;
  }
  .picker {
    position: fixed;
    z-index: 41;
    width: 280px;
    max-height: 320px;
    overflow: auto;
    padding: 4px;
    box-shadow: 0 16px 40px rgba(0, 0, 0, 0.5);
    outline: none;
  }
  .head {
    display: grid;
    gap: 2px;
    padding: 6px 8px 8px;
    border-bottom: 1px solid var(--border);
    margin-bottom: 4px;
  }
  .path {
    font-size: var(--fs-xs);
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  button {
    display: grid;
    gap: 1px;
    width: 100%;
    padding: 7px 8px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text);
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  button.active {
    background: var(--surface-3);
  }
  .name {
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
  }
  .desc {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .none {
    margin: 0;
    padding: 8px;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
