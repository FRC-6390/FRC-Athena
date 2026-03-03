<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faPlus, faXmark } from '@fortawesome/free-solid-svg-icons';
  import { tick } from 'svelte';

  type TabSummary = {
    id: string;
    name: string;
    widgetCount: number;
  };

  type Props = {
    tabs: TabSummary[];
    activeTabId: string;
    onSelectTab: (tabId: string) => void;
    onAddTab: () => void;
    onRemoveTab: (tabId: string) => void;
    onRenameTab: (tabId: string, name: string) => void;
  };

  let { tabs, activeTabId, onSelectTab, onAddTab, onRemoveTab, onRenameTab }: Props = $props();

  let editTabId = $state<string | null>(null);
  let editName = $state('');
  let renameInputEl = $state<HTMLInputElement | null>(null);

  async function beginRename(tabId: string, current: string) {
    editTabId = tabId;
    editName = current;
    await tick();
    renameInputEl?.focus();
    renameInputEl?.select();
  }

  function commitRename(tabId: string) {
    const trimmed = editName.trim();
    if (trimmed) {
      onRenameTab(tabId, trimmed);
    }
    editTabId = null;
    editName = '';
  }
</script>

<section class="tabs-bar panel">
  <div class="tabs-scroll" role="tablist" aria-label="Dashboard workspaces">
    {#each tabs as tab}
      <article class={`tab-chip ${tab.id === activeTabId ? 'active' : ''}`}>
        {#if editTabId === tab.id}
          <input
            class="tab-name-input"
            bind:this={renameInputEl}
            value={editName}
            aria-label="Rename tab"
            oninput={(e) => (editName = (e.currentTarget as HTMLInputElement).value)}
            onkeydown={(e) => {
              if (e.key === 'Enter') commitRename(tab.id);
              if (e.key === 'Escape') {
                editTabId = null;
                editName = '';
              }
            }}
            onblur={() => commitRename(tab.id)}
          />
        {:else}
          <button
            class="tab-name"
            role="tab"
            aria-selected={tab.id === activeTabId}
            onclick={() => onSelectTab(tab.id)}
            ondblclick={() => beginRename(tab.id, tab.name)}
            title="Double-click to rename"
          >
            {tab.name}
          </button>
        {/if}

        <span class="count">{tab.widgetCount}</span>
        <button
          class="tab-icon"
          title="Remove tab"
          aria-label="Remove tab"
          disabled={tabs.length <= 1}
          onclick={() => onRemoveTab(tab.id)}
        >
          <FontAwesomeIcon icon={faXmark} class="tab-icon-glyph" />
        </button>
      </article>
    {/each}
  </div>

  <button class="btn btn-primary tab-add-btn" onclick={onAddTab}>
    <FontAwesomeIcon icon={faPlus} class="tab-add-icon" />
    New
  </button>
</section>

<style>
  .tabs-bar {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.7rem;
    padding: 0.48rem 0.58rem;
  }

  .tabs-scroll {
    display: flex;
    gap: 0.4rem;
    overflow: auto;
    flex: 1;
    padding-bottom: 0.1rem;
  }

  .tab-chip {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    min-height: 1.95rem;
    display: inline-flex;
    align-items: center;
    gap: 0.3rem;
    padding: 0.23rem 0.28rem 0.23rem 0.4rem;
  }

  .tab-chip.active {
    border-color: var(--brand);
    background: var(--brand-soft);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.2);
  }

  .tab-name {
    border: 0;
    padding: 0;
    background: transparent;
    color: var(--text-strong);
    font-family: var(--font-display);
    font-size: 0.79rem;
  }

  .tab-name-input {
    width: 8.5rem;
    font-size: 0.79rem;
    padding: 0.18rem 0.32rem;
    border-radius: 6px;
  }

  .count {
    color: var(--text-soft);
    font-size: 0.72rem;
    min-width: 1.35rem;
    text-align: center;
  }

  .tab-icon {
    width: 1.32rem;
    height: 1.32rem;
    border-radius: 6px;
    border: 1px solid var(--border-subtle);
    background: var(--surface-3);
    color: var(--text-soft);
    padding: 0;
    display: inline-grid;
    place-items: center;
  }

  .tab-icon:hover:not(:disabled) {
    border-color: var(--border-emphasis);
    color: var(--text-strong);
  }

  .tab-icon-glyph {
    width: 0.64rem;
    height: 0.64rem;
    display: block;
  }

  .tab-add-btn {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
  }

  .tab-add-icon {
    width: 0.58rem;
    height: 0.58rem;
    display: block;
  }
</style>
