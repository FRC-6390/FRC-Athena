<script lang="ts">
  type Props = {
    host: string;
    controlPort: string;
    connected: boolean;
    status: string;
    serverLayoutName: string;
    availableServerLayouts: string[];
    onHostInput: (value: string) => void;
    onPortInput: (value: string) => void;
    onServerLayoutNameInput: (value: string) => void;
    onConnect: () => void;
    onDisconnect: () => void;
    onSaveServerLayout: () => void;
    onLoadServerLayout: () => void;
    onRefreshServerLayouts: () => void;
  };

  let {
    host,
    controlPort,
    connected,
    status,
    serverLayoutName,
    availableServerLayouts,
    onHostInput,
    onPortInput,
    onServerLayoutNameInput,
    onConnect,
    onDisconnect,
    onSaveServerLayout,
    onLoadServerLayout,
    onRefreshServerLayouts
  }: Props = $props();
</script>

<section class="settings-screen panel">
  <header>
    <h2>Connection Settings</h2>
    <p>Configure ARCP control endpoint for this dashboard client.</p>
  </header>

  <div class="settings-grid">
    <label>
      Host
      <input
        value={host}
        placeholder="127.0.0.1"
        oninput={(e) => onHostInput((e.currentTarget as HTMLInputElement).value)}
      />
    </label>

    <label>
      Control port
      <input
        value={controlPort}
        inputmode="numeric"
        placeholder="5805"
        oninput={(e) => onPortInput((e.currentTarget as HTMLInputElement).value)}
      />
    </label>

    <div class="actions">
      <button class="btn btn-primary" onclick={onConnect}>Connect</button>
      <button class="btn" onclick={onDisconnect}>Disconnect</button>
    </div>
  </div>

  <div class="status-row">
    <span class="status-label">Link status</span>
    <span class="badge" data-online={connected}>{connected ? 'ONLINE' : 'OFFLINE'}</span>
    <code>{status}</code>
  </div>

  <div class="server-layout panel">
    <header>
      <h3>Server UI Layout</h3>
      <p>Save/load layout profiles directly on the connected ARCP server.</p>
    </header>
    <div class="layout-controls">
      <label>
        Layout name
        <input
          value={serverLayoutName}
          placeholder="sim-all-layouts"
          oninput={(e) => onServerLayoutNameInput((e.currentTarget as HTMLInputElement).value)}
        />
      </label>
      <div class="actions">
        <button class="btn" onclick={onRefreshServerLayouts} disabled={!connected}>Refresh</button>
        <button class="btn" onclick={onLoadServerLayout} disabled={!connected}>Load</button>
        <button class="btn btn-primary" onclick={onSaveServerLayout} disabled={!connected}>Save</button>
      </div>
    </div>
    {#if availableServerLayouts.length > 0}
      <div class="layout-list" aria-label="Available server layouts">
        {#each availableServerLayouts as name}
          <button
            class="layout-chip"
            onclick={() => onServerLayoutNameInput(name)}
            title={`Use layout '${name}'`}
          >
            {name}
          </button>
        {/each}
      </div>
    {:else}
      <p class="layout-empty">No server layouts discovered yet.</p>
    {/if}
  </div>

</section>

<style>
  .settings-screen {
    padding: 0.9rem;
    display: grid;
    grid-template-rows: auto auto auto auto;
    gap: 0.86rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 1rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.2rem 0 0;
    font-size: 0.78rem;
    color: var(--text-soft);
  }

  .settings-grid {
    display: grid;
    grid-template-columns: minmax(180px, 280px) minmax(120px, 180px) auto;
    gap: 0.6rem;
    align-items: end;
  }

  label {
    display: grid;
    gap: 0.2rem;
    font-size: 0.75rem;
    color: var(--text-soft);
  }

  .actions {
    display: flex;
    gap: 0.45rem;
  }

  .status-row {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 0.58rem 0.66rem;
    background: var(--surface-2);
  }

  .status-label {
    color: var(--text-soft);
    font-size: 0.74rem;
  }

  code {
    font-family: var(--font-mono);
    font-size: 0.72rem;
    color: var(--text);
    background: var(--surface-3);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 0.2rem 0.35rem;
    overflow: auto;
    white-space: nowrap;
  }

  @media (max-width: 960px) {
    .settings-grid {
      grid-template-columns: 1fr;
      align-items: stretch;
    }

    .actions {
      justify-content: flex-start;
    }

    .status-row {
      flex-wrap: wrap;
    }
  }

  .server-layout {
    padding: 0.72rem;
    display: grid;
    gap: 0.6rem;
    border-radius: 10px;
  }

  .server-layout header h3 {
    margin: 0;
    font-size: 0.9rem;
    color: var(--text-strong);
  }

  .server-layout header p {
    margin: 0.15rem 0 0;
    font-size: 0.74rem;
    color: var(--text-soft);
  }

  .layout-controls {
    display: grid;
    grid-template-columns: minmax(160px, 260px) auto;
    gap: 0.5rem;
    align-items: end;
  }

  .layout-list {
    display: flex;
    flex-wrap: wrap;
    gap: 0.4rem;
  }

  .layout-chip {
    border: 1px solid var(--border-subtle);
    background: var(--surface-3);
    border-radius: 999px;
    padding: 0.24rem 0.55rem;
    color: var(--text);
    font-size: 0.73rem;
    cursor: pointer;
  }

  .layout-chip:hover {
    border-color: var(--brand);
    color: var(--brand-contrast);
    background: color-mix(in srgb, var(--brand) 12%, transparent);
  }

  .layout-empty {
    margin: 0;
    font-size: 0.73rem;
    color: var(--text-soft);
  }

  @media (max-width: 960px) {
    .layout-controls {
      grid-template-columns: 1fr;
    }
  }
</style>
