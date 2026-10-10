<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faClipboard } from '@fortawesome/free-solid-svg-icons';
  import { widgetMenuItems, widgetMenuStyle, type WidgetMenuAction, type WidgetMenuState } from '../../lib/canvas/context-menu';

  let {
    menu,
    copyToastVisible,
    onAction
  }: {
    menu: WidgetMenuState | null;
    copyToastVisible: boolean;
    onAction: (action: WidgetMenuAction) => void;
  } = $props();

  function viewport(): { width: number; height: number } {
    if (typeof window === 'undefined') return { width: 1920, height: 1080 };
    return { width: window.innerWidth, height: window.innerHeight };
  }
</script>

{#if menu}
  <div class="widget-context-menu" style={widgetMenuStyle(menu, viewport())} role="menu" aria-label="Widget actions">
    {#each widgetMenuItems(menu) as item (item.action)}
      <button
        class={`menu-item ${item.danger ? 'danger' : ''}`}
        role="menuitem"
        onclick={(event) => {
          event.preventDefault();
          event.stopPropagation();
          onAction(item.action);
        }}
      >
        {item.label}
        <span>{item.shortcut}</span>
      </button>
    {/each}
  </div>
{/if}

{#if copyToastVisible}
  <div class="copy-toast" role="status" aria-live="polite">
    <FontAwesomeIcon icon={faClipboard} class="copy-toast-icon" />
    <span>Copied</span>
  </div>
{/if}

<style>
  .widget-context-menu {
    position: fixed;
    z-index: 2000;
    width: 210px;
    border: 1px solid var(--border-subtle);
    border-radius: 9px;
    background: #1b2230;
    box-shadow: 0 14px 32px rgba(0, 0, 0, 0.45);
    padding: 0.2rem;
    display: grid;
    gap: 0.1rem;
  }

  .menu-item {
    border: 1px solid transparent;
    border-radius: 6px;
    background: transparent;
    color: var(--text);
    padding: 0.43rem 0.52rem;
    font-size: 0.74rem;
    text-align: left;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  .menu-item span {
    color: var(--text-soft);
    font-family: var(--font-mono);
    font-size: 0.68rem;
  }

  .menu-item:hover {
    border-color: var(--border-subtle);
    background: #273146;
  }

  .menu-item.danger {
    color: #fecaca;
  }

  .menu-item.danger:hover {
    border-color: rgba(248, 113, 113, 0.45);
    background: rgba(127, 29, 29, 0.36);
  }

  .copy-toast {
    position: absolute;
    left: 50%;
    bottom: 0.78rem;
    transform: translateX(-50%);
    z-index: 2101;
    pointer-events: none;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.46rem;
    min-width: 9.6rem;
    border: 1px solid rgba(239, 68, 68, 0.75);
    border-radius: 12px;
    background: linear-gradient(180deg, rgba(185, 28, 28, 0.97), rgba(127, 29, 29, 0.97));
    color: #fff1f2;
    padding: 0.52rem 0.96rem;
    font-size: 0.92rem;
    font-weight: 600;
    line-height: 1;
    box-shadow:
      0 10px 24px rgba(0, 0, 0, 0.4),
      inset 0 1px 0 rgba(255, 255, 255, 0.12);
  }

  .copy-toast :global(.copy-toast-icon) {
    width: 0.88rem;
    height: 0.88rem;
    display: block;
    color: #ffe4e6;
  }
</style>
