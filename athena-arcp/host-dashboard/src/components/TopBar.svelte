<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faFileArrowDown, faFileArrowUp } from '@fortawesome/free-solid-svg-icons';

  type Props = {
    connected: boolean;
    presentationMode: boolean;
    driverstationDockMode: boolean;
    dashboardStates: Array<{ key: string; label: string; value: string; valueWidthCh?: number }>;
    onLoadLayout: () => void;
    onSaveLayout: () => void;
    onTogglePresentationMode: () => void;
    onToggleDriverstationDockMode: () => void;
  };

  let {
    connected,
    presentationMode,
    driverstationDockMode,
    dashboardStates,
    onLoadLayout,
    onSaveLayout,
    onTogglePresentationMode,
    onToggleDriverstationDockMode
  }: Props = $props();
</script>

<header class="topbar panel">
  <div class="toolbar-actions">
    <div class="state-strip" aria-label="Dashboard state">
      {#each dashboardStates as state (state.key)}
        <span class="state-pill" style={`--value-ch:${state.valueWidthCh ?? 7.5};`}>
          <span class="state-label">{state.label}</span>
          <span class="state-value">{state.value}</span>
        </span>
      {/each}
    </div>
    <div class="window-tools">
      <button class="btn btn-mini icon-btn" onclick={onLoadLayout} title="Load UI (Ctrl+O)">
        <FontAwesomeIcon icon={faFileArrowUp} />
        <span>Load UI</span>
      </button>
      <button class="btn btn-mini icon-btn" onclick={onSaveLayout} title="Save UI (Ctrl+S)">
        <FontAwesomeIcon icon={faFileArrowDown} />
        <span>Save UI</span>
      </button>
      <button class="btn btn-mini" data-active={presentationMode} onclick={onTogglePresentationMode}>
        Fullscreen
      </button>
      <button
        class="btn btn-mini"
        data-active={driverstationDockMode}
        onclick={onToggleDriverstationDockMode}
      >
        Dock DS
      </button>
    </div>
  </div>

  <div class="badge" data-online={connected}>{connected ? 'ONLINE' : 'OFFLINE'}</div>
</header>

<style>
  .topbar {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.8rem;
    padding: 0.55rem 0.74rem;
    background: #171d28;
    border-color: var(--border);
    min-width: 0;
    overflow: hidden;
  }

  .toolbar-actions {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    justify-content: stretch;
    min-width: 0;
  }

  .window-tools {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
  }

  .icon-btn {
    display: inline-flex;
    align-items: center;
    gap: 0.28rem;
  }

  .icon-btn :global(svg) {
    width: 0.68rem;
    height: 0.68rem;
    display: block;
  }

  .state-strip {
    display: flex;
    gap: 0.26rem;
    flex-wrap: wrap;
    justify-content: flex-start;
    align-items: center;
    flex: 1 1 auto;
    min-width: 0;
  }

  .state-pill {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: var(--surface-2);
    display: inline-grid;
    grid-template-columns: auto auto;
    align-items: center;
    gap: 0.24rem;
    min-width: 0;
    line-height: 1;
    padding: 0.22rem 0.34rem;
    white-space: nowrap;
  }

  .state-label {
    color: var(--text-soft);
    font-size: 0.7rem;
    text-align: left;
    min-width: 0;
  }

  .state-value {
    color: var(--text);
    font-size: 0.7rem;
    text-align: left;
    width: calc(var(--value-ch, 7.5) * 1ch);
    font-variant-numeric: tabular-nums;
    font-feature-settings: 'tnum' 1;
    font-family: var(--font-mono);
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .toolbar-actions .btn[data-active='true'] {
    border-color: rgba(180, 35, 45, 0.5);
    background: rgba(180, 35, 45, 0.24);
    color: #ffe7ea;
  }

  .topbar :global(.btn) {
    border-color: var(--border-subtle);
    background: #242d3d;
    color: var(--text);
  }

  .topbar :global(.btn.btn-primary) {
    border-color: var(--brand);
    background: var(--brand);
    color: #fff;
  }

  .badge {
    min-width: 84px;
    justify-self: end;
  }

  @media (max-width: 860px) {
    .topbar {
      grid-template-columns: 1fr;
      align-items: stretch;
    }

    .toolbar-actions {
      flex-wrap: wrap;
    }

    .badge {
      justify-self: start;
    }
  }
</style>
