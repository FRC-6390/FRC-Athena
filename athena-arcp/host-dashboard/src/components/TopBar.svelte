<script lang="ts">
  import { onMount } from 'svelte';
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faChevronDown, faFileArrowDown, faFileArrowUp } from '@fortawesome/free-solid-svg-icons';

  type Props = {
    connected: boolean;
    stale: boolean;
    connecting: boolean;
    serverLayoutName: string;
    availableServerLayouts: string[];
    recordingActive: boolean;
    replayActive: boolean;
    recordingStatus: string;
    recordings: Array<{ id: string; name: string }>;
    selectedRecordingId: string;
    presentationMode: boolean;
    driverstationDockMode: boolean;
    dashboardStates: Array<{ key: string; label: string; value: string; valueWidthCh?: number }>;
    onLoadLayoutFromHost: () => void;
    onLoadLayoutFromServer: () => void;
    onSaveLayoutToHost: () => void;
    onSaveLayoutToServer: () => void;
    onServerLayoutNameInput: (value: string) => void;
    onToggleRecording: () => void;
    onSelectedRecordingIdInput: (recordingId: string) => void;
    onLoadSelectedRecording: () => void;
    onImportRecording: () => void;
    onStopReplay: () => void;
    onRefreshServerLayouts: () => void;
    onTogglePresentationMode: () => void;
    onToggleDriverstationDockMode: () => void;
    onConnectionBadgeClick: () => void;
  };

  let {
    connected,
    stale,
    connecting,
    serverLayoutName,
    availableServerLayouts,
    recordingActive,
    replayActive,
    recordingStatus,
    recordings,
    selectedRecordingId,
    presentationMode,
    driverstationDockMode,
    dashboardStates,
    onLoadLayoutFromHost,
    onLoadLayoutFromServer,
    onSaveLayoutToHost,
    onSaveLayoutToServer,
    onServerLayoutNameInput,
    onToggleRecording,
    onSelectedRecordingIdInput,
    onLoadSelectedRecording,
    onImportRecording,
    onStopReplay,
    onRefreshServerLayouts,
    onTogglePresentationMode,
    onToggleDriverstationDockMode,
    onConnectionBadgeClick
  }: Props = $props();

  let menuRootEl = $state<HTMLElement | null>(null);
  let loadMenuOpen = $state(false);
  let saveMenuOpen = $state(false);
  let recordingMenuOpen = $state(false);

  function closeMenus() {
    loadMenuOpen = false;
    saveMenuOpen = false;
    recordingMenuOpen = false;
  }

  function toggleLoadMenu() {
    loadMenuOpen = !loadMenuOpen;
    if (loadMenuOpen) {
      saveMenuOpen = false;
      onRefreshServerLayouts();
    }
  }

  function toggleSaveMenu() {
    saveMenuOpen = !saveMenuOpen;
    if (saveMenuOpen) {
      loadMenuOpen = false;
      recordingMenuOpen = false;
      onRefreshServerLayouts();
    }
  }

  function toggleRecordingMenu() {
    recordingMenuOpen = !recordingMenuOpen;
    if (recordingMenuOpen) {
      loadMenuOpen = false;
      saveMenuOpen = false;
    }
  }

  onMount(() => {
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target as HTMLElement | null;
      if (target && menuRootEl?.contains(target)) {
        return;
      }
      closeMenus();
    };
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        closeMenus();
      }
    };
    window.addEventListener('pointerdown', onPointerDown);
    window.addEventListener('keydown', onKeyDown);
    return () => {
      window.removeEventListener('pointerdown', onPointerDown);
      window.removeEventListener('keydown', onKeyDown);
    };
  });
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
    <div class="window-tools" bind:this={menuRootEl}>
      <div class="menu-wrap">
        <div class="recording-controls-inline">
          <button
            class={`btn btn-mini ${recordingActive ? 'btn-primary' : ''}`}
            onclick={onToggleRecording}
            disabled={!recordingActive && replayActive}
            title={recordingActive
              ? 'Stop active recording'
              : replayActive
                ? 'Stop replay before starting a recording'
                : 'Start recording telemetry'}
          >
            {recordingActive ? 'Stop Rec' : 'Start Rec'}
          </button>
          <button
            class="btn btn-mini icon-btn menu-trigger recording-menu-trigger"
            data-open={recordingMenuOpen}
            onclick={toggleRecordingMenu}
            title="Recording menu"
            aria-label="Open recording menu"
          >
            <span class="chevron"><FontAwesomeIcon icon={faChevronDown} /></span>
          </button>
        </div>
        {#if recordingMenuOpen}
          <div class="menu-card panel">
            <label class="menu-label">
              Saved recordings
              <select
                class="menu-select"
                value={selectedRecordingId}
                onchange={(e) => onSelectedRecordingIdInput((e.currentTarget as HTMLSelectElement).value)}
                disabled={recordings.length === 0}
              >
                {#if recordings.length === 0}
                  <option value="">No recordings</option>
                {:else}
                  {#each recordings as recording}
                    <option value={recording.id}>{recording.name}</option>
                  {/each}
                {/if}
              </select>
            </label>
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onLoadSelectedRecording();
              }}
              disabled={recordings.length === 0 || !selectedRecordingId || recordingActive}
              title={recordingActive ? 'Stop active recording before loading replay' : 'Load selected recording'}
            >
              Load Selected Recording
            </button>
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onImportRecording();
              }}
            >
              Load Recording File
            </button>
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onStopReplay();
              }}
              disabled={!replayActive}
            >
              Stop Replay
            </button>
            <p class="menu-note">{recordingStatus}</p>
          </div>
        {/if}
      </div>

      <div class="menu-wrap">
        <button
          class="btn btn-mini icon-btn menu-trigger"
          data-open={loadMenuOpen}
          onclick={toggleLoadMenu}
          title="Load UI (Ctrl+O)"
        >
          <FontAwesomeIcon icon={faFileArrowUp} />
          <span>Load UI</span>
          <span class="chevron"><FontAwesomeIcon icon={faChevronDown} /></span>
        </button>
        {#if loadMenuOpen}
          <div class="menu-card panel">
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onLoadLayoutFromHost();
              }}
            >
              Load from Host File
            </button>
            <label class="menu-label">
              Server profile
              <select
                class="menu-select"
                value={serverLayoutName}
                onchange={(e) => onServerLayoutNameInput((e.currentTarget as HTMLSelectElement).value)}
                disabled={!connected || availableServerLayouts.length === 0}
              >
                {#if availableServerLayouts.length === 0}
                  <option value="">No server layouts</option>
                {:else}
                  {#each availableServerLayouts as name}
                    <option value={name}>{name}</option>
                  {/each}
                {/if}
              </select>
            </label>
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onLoadLayoutFromServer();
              }}
              disabled={!connected || !serverLayoutName || availableServerLayouts.length === 0}
              title={connected && serverLayoutName
                ? `Load '${serverLayoutName}' from server`
                : 'Connect and select a server profile to load'}
            >
              Load from Server
            </button>
          </div>
        {/if}
      </div>

      <div class="menu-wrap">
        <button
          class="btn btn-mini icon-btn menu-trigger"
          data-open={saveMenuOpen}
          onclick={toggleSaveMenu}
          title="Save UI (Ctrl+S)"
        >
          <FontAwesomeIcon icon={faFileArrowDown} />
          <span>Save UI</span>
          <span class="chevron"><FontAwesomeIcon icon={faChevronDown} /></span>
        </button>
        {#if saveMenuOpen}
          <div class="menu-card panel">
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onSaveLayoutToHost();
              }}
            >
              Save to Host File
            </button>
            <label class="menu-label">
              Server profile
              <input
                class="menu-input"
                value={serverLayoutName}
                placeholder="sim-all-layouts"
                oninput={(e) => onServerLayoutNameInput((e.currentTarget as HTMLInputElement).value)}
              />
            </label>
            <button
              class="menu-item"
              onclick={() => {
                closeMenus();
                onSaveLayoutToServer();
              }}
              disabled={!connected}
              title={connected ? `Save to '${serverLayoutName}' on server` : 'Connect to save to server'}
            >
              Save to Server
            </button>
          </div>
        {/if}
      </div>
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

  <button
    type="button"
    class="badge badge-button"
    data-state={connected ? 'online' : connecting ? 'reconnecting' : stale ? 'stale' : 'offline'}
    title={connected
      ? 'Connected'
      : 'Retry connection with current host and port'}
    onclick={onConnectionBadgeClick}
  >
    {connected ? 'ONLINE' : connecting ? 'RETRYING' : stale ? 'STALE' : 'OFFLINE'}
  </button>
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
    overflow: visible;
    position: relative;
    z-index: 80;
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

  .menu-wrap {
    position: relative;
  }

  .recording-controls-inline {
    display: inline-flex;
    align-items: center;
    gap: 0.22rem;
  }

  .recording-menu-trigger {
    width: 1.62rem;
    min-width: 1.62rem;
    padding-left: 0.26rem;
    padding-right: 0.26rem;
  }

  .menu-note {
    margin: 0;
    padding: 0.24rem 0.26rem 0.1rem;
    color: var(--text-soft);
    font-size: 0.68rem;
    line-height: 1.25;
  }

  .menu-trigger {
    padding-right: 0.42rem;
  }

  .menu-trigger[data-open='true'] {
    border-color: rgba(180, 35, 45, 0.5);
    background: rgba(180, 35, 45, 0.2);
  }

  .chevron {
    width: 0.55rem;
    height: 0.55rem;
    opacity: 0.9;
    display: inline-grid;
    place-items: center;
  }

  .chevron :global(svg) {
    width: 0.55rem;
    height: 0.55rem;
  }

  .menu-card {
    position: absolute;
    top: calc(100% + 0.32rem);
    right: 0;
    z-index: 200;
    min-width: 220px;
    padding: 0.24rem;
    display: grid;
    gap: 0.18rem;
    background: #1b2331;
    border-color: var(--border);
  }

  .menu-item {
    border: 1px solid var(--border-subtle);
    background: #252f41;
    color: var(--text);
    border-radius: 7px;
    padding: 0.46rem 0.56rem;
    text-align: left;
    font-size: 0.73rem;
    cursor: pointer;
  }

  .menu-item:hover {
    border-color: rgba(180, 35, 45, 0.55);
    background: rgba(180, 35, 45, 0.2);
  }

  .menu-item:disabled {
    opacity: 0.45;
    cursor: not-allowed;
  }

  .menu-label {
    display: grid;
    gap: 0.24rem;
    font-size: 0.71rem;
    color: var(--text-soft);
    padding: 0.1rem 0.1rem 0.16rem;
  }

  .menu-input,
  .menu-select {
    border: 1px solid var(--border-subtle);
    background: #232c3d;
    color: var(--text);
    border-radius: 7px;
    font-size: 0.73rem;
    padding: 0.34rem 0.44rem;
    min-width: 0;
  }

  .menu-input:focus,
  .menu-select:focus {
    outline: none;
    border-color: rgba(180, 35, 45, 0.62);
    box-shadow: 0 0 0 1px rgba(180, 35, 45, 0.28);
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

  .badge-button {
    cursor: pointer;
    transition: transform 120ms ease, filter 120ms ease;
  }

  .badge-button:hover {
    filter: brightness(1.06);
  }

  .badge-button:active {
    transform: translateY(1px);
  }

  .badge[data-state='online'] {
    border-color: rgba(72, 170, 106, 0.52);
    background: rgba(40, 120, 76, 0.24);
    color: #d8ffe6;
  }

  .badge[data-state='reconnecting'] {
    border-color: rgba(122, 148, 198, 0.52);
    background: rgba(44, 73, 124, 0.24);
    color: #dfe9ff;
  }

  .badge[data-state='stale'] {
    border-color: rgba(189, 133, 54, 0.52);
    background: rgba(130, 89, 29, 0.24);
    color: #ffeaca;
  }

  .badge[data-state='offline'] {
    border-color: rgba(180, 35, 45, 0.52);
    background: rgba(120, 26, 31, 0.24);
    color: #ffd9dd;
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
