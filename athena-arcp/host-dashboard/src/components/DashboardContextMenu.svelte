<script lang="ts">
  type Props = {
    style: string;
    canClearLayout: boolean;
    canRenamePage: boolean;
    onImportLayout: () => void;
    onExportLayout: () => void;
    onOpenDataExplorer: () => void;
    onOpenLayoutTools: () => void;
    onAddPage: () => void;
    onRenamePage: () => void;
    onClearLayout: () => void;
  };

  let {
    style,
    canClearLayout,
    canRenamePage,
    onImportLayout,
    onExportLayout,
    onOpenDataExplorer,
    onOpenLayoutTools,
    onAddPage,
    onRenamePage,
    onClearLayout
  }: Props = $props();
</script>

<div class="dashboard-context-menu" style={style} role="menu" aria-label="Dashboard actions">
  <button class="menu-item" role="menuitem" onclick={onOpenDataExplorer}>
    Open data explorer
  </button>
  <button class="menu-item" role="menuitem" onclick={onOpenLayoutTools}>
    Open layout tools
  </button>
  <button class="menu-item" role="menuitem" onclick={onImportLayout}>
    Load UI JSON
  </button>
  <button class="menu-item" role="menuitem" onclick={onExportLayout}>
    Save UI JSON
  </button>

  <div class="separator" aria-hidden="true"></div>

  <button class="menu-item" role="menuitem" onclick={onAddPage}>
    Add page
  </button>
  <button class="menu-item" role="menuitem" disabled={!canRenamePage} onclick={onRenamePage}>
    Rename page
  </button>
  <button class="menu-item danger" role="menuitem" disabled={!canClearLayout} onclick={onClearLayout}>
    Clear layout
  </button>
</div>

<style>
  .dashboard-context-menu {
    position: fixed;
    z-index: 2100;
    width: 222px;
    border: 1px solid var(--border-subtle);
    border-radius: 9px;
    background: #1b2230;
    box-shadow: 0 14px 32px rgba(0, 0, 0, 0.45);
    padding: 0.2rem;
    display: grid;
    gap: 0.12rem;
  }

  .separator {
    margin: 0.04rem 0.1rem;
    border-top: 1px solid var(--border-subtle);
    opacity: 0.8;
  }

  .menu-item {
    border: 1px solid transparent;
    border-radius: 6px;
    background: transparent;
    color: var(--text);
    padding: 0.42rem 0.5rem;
    font-size: 0.74rem;
    text-align: left;
  }

  .menu-item:hover:not(:disabled) {
    border-color: var(--border-subtle);
    background: #273146;
  }

  .menu-item:disabled {
    opacity: 0.46;
    cursor: not-allowed;
  }

  .menu-item.danger {
    color: #fecaca;
  }

  .menu-item.danger:hover:not(:disabled) {
    border-color: rgba(248, 113, 113, 0.45);
    background: rgba(127, 29, 29, 0.36);
  }
</style>
