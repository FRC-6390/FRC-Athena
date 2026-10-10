<script lang="ts">
  import type { Session } from '../lib/data/session';
  import { buildMechanismTree, flattenMechanisms } from '../lib/data/tree';
  import { summarize } from '../lib/mechanisms/model';
  import { formatClock, type DriverState } from '../lib/driver/driver-state';
  import { reasonTone } from './mechanisms/tones';
  import { classicCommands, toClassicRows } from '../lib/canvas-bridge';
  import {
    browserLayoutStorage,
    cleanLayoutName,
    createLayoutStore
  } from '../lib/canvas-layouts';
  import { downloadJsonFile, pickJsonFileText } from '../lib/browser-io';
  import type { DashboardLayoutState } from '../lib/dashboard';
  import CanvasDashboards from './canvas/CanvasDashboards.svelte';

  type Props = {
    session: Session;
    live: { readonly current: number };
    driver: DriverState;
    onmechanism: (path: string) => void;
  };

  let { session, live, driver, onmechanism }: Props = $props();

  const OVERVIEW = 'overview';
  const store = createLayoutStore(browserLayoutStorage());

  let names = $state<string[]>(store.list());
  let active = $state(OVERVIEW);
  let layoutDraft = $state<DashboardLayoutState | null>(null);
  let editing = $state(false);

  let creating = $state(false);
  let newName = $state('');
  let renaming = $state(false);
  let renameTo = $state('');
  let menuOpen = $state(false);
  let confirmDelete = $state(false);
  let message = $state('');
  let messageTone = $state<'ok' | 'fault'>('ok');

  // Canvas rows and commands only exist while a dashboard tab is open.
  const rows = $derived.by(() => {
    void live.current;
    return active === OVERVIEW ? [] : toClassicRows(session.store);
  });
  const commands = $derived(classicCommands(session.store, session.commands));

  const mechanisms = $derived.by(() => {
    void live.current;
    const roots = buildMechanismTree(session.store.allSignals());
    const robot = roots.find((r) => r.name === 'robot');
    return flattenMechanisms(robot ? robot.children : roots).map((node) => {
      const summary = summarize(node);
      const reason = summary.reason ? String(session.store.latestSample(summary.reason.signalId)?.value ?? '') : '';
      return { node, reason };
    });
  });

  const phaseLabel = $derived(
    { idle: 'Pre-match', auto: 'Autonomous', teleop: 'Teleop', ended: 'Match over' }[driver.match.phase]
  );
  const clock = $derived(
    driver.match.remainingSeconds !== null ? driver.match.remainingSeconds : driver.match.elapsedSeconds
  );
  const voltsPct = $derived(
    driver.battery.volts === null ? 0 : Math.max(0, Math.min(1, (driver.battery.volts - 10) / 3)) * 100
  );
  const batteryTone = $derived(
    driver.battery.volts === null ? 'stale' : driver.battery.volts < 11 ? 'fault' : driver.battery.volts < 12 ? 'warn' : 'ok'
  );

  function say(text: string, tone: 'ok' | 'fault' = 'ok') {
    message = text;
    messageTone = tone;
  }

  function resetTransient() {
    menuOpen = false;
    confirmDelete = false;
    renaming = false;
    message = '';
  }

  function select(key: string) {
    resetTransient();
    if (key === OVERVIEW) {
      active = OVERVIEW;
      layoutDraft = null;
      editing = false;
      return;
    }
    const loaded = store.load(key);
    if (!loaded) {
      say(`"${key}" could not be loaded`, 'fault');
      names = store.list();
      active = OVERVIEW;
      layoutDraft = null;
      return;
    }
    active = key;
    layoutDraft = loaded;
    editing = false;
  }

  function saveActive(next: DashboardLayoutState) {
    if (active !== OVERVIEW) store.save(active, next);
  }

  function startCreate() {
    resetTransient();
    creating = true;
    newName = '';
  }

  function cancelCreate() {
    creating = false;
    newName = '';
  }

  function createLayout(event: SubmitEvent) {
    event.preventDefault();
    const clean = cleanLayoutName(newName);
    if (!clean) return;
    if (store.list().includes(clean)) {
      say(`A dashboard named "${clean}" already exists`, 'fault');
      return;
    }
    const stored = store.create(clean);
    names = store.list();
    creating = false;
    newName = '';
    select(stored);
    editing = true;
  }

  function startRename() {
    renaming = true;
    renameTo = active;
    menuOpen = false;
    message = '';
  }

  function commitRename(event: SubmitEvent) {
    event.preventDefault();
    const clean = cleanLayoutName(renameTo);
    if (!clean) return;
    if (!store.rename(active, clean)) {
      say(clean === active ? '' : `"${clean}" is taken`, 'fault');
      return;
    }
    names = store.list();
    renaming = false;
    active = clean;
    say(`renamed to "${clean}"`);
  }

  function duplicate() {
    const copy = store.duplicate(active);
    menuOpen = false;
    if (!copy) return;
    names = store.list();
    select(copy);
    say(`duplicated as "${copy}"`);
  }

  function exportActive() {
    const text = store.exportText(active);
    menuOpen = false;
    if (!text) return;
    const slug = active.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-+|-+$/g, '') || 'dashboard';
    downloadJsonFile(`arcp-dashboard-${slug}.json`, text);
    say(`exported "${active}"`);
  }

  async function importLayout() {
    menuOpen = false;
    try {
      const text = await pickJsonFileText();
      if (!text) return;
      const stored = store.importText(text, 'Imported dashboard');
      names = store.list();
      select(stored);
      say(`imported as "${stored}"`);
    } catch (err) {
      say(`import failed: ${err instanceof Error ? err.message : String(err)}`, 'fault');
    }
  }

  function askDelete() {
    confirmDelete = true;
  }

  function deleteActive() {
    const removed = active;
    store.remove(removed);
    names = store.list();
    select(OVERVIEW);
    say(`deleted "${removed}"`);
  }

  function onWindowPointerDown(event: PointerEvent) {
    if (!menuOpen) return;
    const target = event.target as HTMLElement | null;
    if (target?.closest('.menu-wrap')) return;
    menuOpen = false;
    confirmDelete = false;
  }

  function onWindowKeyDown(event: KeyboardEvent) {
    if (event.key !== 'Escape') return;
    if (menuOpen) {
      menuOpen = false;
      confirmDelete = false;
    }
    if (renaming) renaming = false;
    if (creating) cancelCreate();
  }

  function focusOnMount(node: HTMLInputElement) {
    node.focus();
    node.select();
  }
</script>

<svelte:window onpointerdown={onWindowPointerDown} onkeydown={onWindowKeyDown} />

<div class="screen">
  <div class="strip">
    <div class="tabs" role="tablist">
      <button type="button" role="tab" class:active={active === OVERVIEW} aria-selected={active === OVERVIEW} onclick={() => select(OVERVIEW)}>
        Overview
      </button>
      {#each names as name (name)}
        <button type="button" role="tab" class:active={active === name} aria-selected={active === name} onclick={() => select(name)}>
          {name}
        </button>
      {/each}

      {#if creating}
        <form class="inline" onsubmit={createLayout}>
          <input
            class="ui-input"
            bind:value={newName}
            placeholder="Dashboard name"
            aria-label="New dashboard name"
            maxlength="40"
            use:focusOnMount
          />
          <button class="ui-btn" data-variant="primary" data-size="sm" type="submit" disabled={!cleanLayoutName(newName)}>Create</button>
          <button class="ui-btn" data-variant="ghost" data-size="sm" type="button" onclick={cancelCreate}>Cancel</button>
        </form>
      {:else}
        <button class="ui-btn new" data-variant="ghost" data-size="sm" type="button" onclick={startCreate}>+ New dashboard</button>
      {/if}

      <span class="ui-spacer"></span>

      {#if active !== OVERVIEW && layoutDraft}
        <button
          class="ui-btn"
          data-variant={editing ? 'primary' : 'ghost'}
          data-size="sm"
          type="button"
          aria-pressed={editing}
          onclick={() => (editing = !editing)}
        >
          {editing ? 'Done' : 'Edit'}
        </button>

        <div class="menu-wrap">
          <button
            class="ui-btn"
            data-variant="ghost"
            data-size="sm"
            type="button"
            aria-label="Dashboard actions"
            aria-haspopup="menu"
            aria-expanded={menuOpen}
            onclick={() => {
              menuOpen = !menuOpen;
              confirmDelete = false;
            }}
          >
            &hellip;
          </button>
          {#if menuOpen}
            <div class="menu" role="menu">
              <button type="button" role="menuitem" onclick={startRename}>Rename</button>
              <button type="button" role="menuitem" onclick={duplicate}>Duplicate</button>
              <button type="button" role="menuitem" onclick={exportActive}>Export JSON</button>
              <button type="button" role="menuitem" onclick={importLayout}>Import JSON</button>
              {#if confirmDelete}
                <button type="button" role="menuitem" class="danger" onclick={deleteActive}>Confirm delete</button>
              {:else}
                <button type="button" role="menuitem" class="danger" onclick={askDelete}>Delete</button>
              {/if}
            </div>
          {/if}
        </div>
      {/if}
    </div>

    {#if renaming}
      <form class="rename" onsubmit={commitRename}>
        <label class="ui-label" for="rename-dashboard">Rename dashboard</label>
        <input id="rename-dashboard" class="ui-input" bind:value={renameTo} maxlength="40" use:focusOnMount />
        <button class="ui-btn" data-variant="primary" data-size="sm" type="submit">Save</button>
        <button class="ui-btn" data-variant="ghost" data-size="sm" type="button" onclick={() => (renaming = false)}>Cancel</button>
      </form>
    {/if}

    {#if message}
      <p class="message" data-tone={messageTone} role="status">{message}</p>
    {/if}
  </div>

  {#if active !== OVERVIEW && layoutDraft}
    <div class="canvas-host">
      {#key active}
        <CanvasDashboards bind:layout={layoutDraft} {rows} {commands} bind:editing onLayoutChange={saveActive} />
      {/key}
    </div>
  {:else}
    <div class="grid">
      <section class="ui-panel tile match">
        <header class="ui-panel-head"><h3 class="ui-panel-title">Match</h3></header>
        <div class="ui-panel-body match-body">
          <div class="clock">{formatClock(clock)}</div>
          <div class="phase">{phaseLabel}</div>
          <div class="mode ui-badge">{driver.mode ?? 'No mode'}</div>
        </div>
      </section>

      <section class="ui-panel tile">
        <header class="ui-panel-head"><h3 class="ui-panel-title">Battery</h3></header>
        <div class="ui-panel-body battery">
          <div class="big ui-mono" data-tone={batteryTone}>
            {driver.battery.volts === null ? '--.-' : driver.battery.volts.toFixed(2)}<span>V</span>
          </div>
          <div class="meter"><i style="width:{voltsPct}%" data-tone={batteryTone}></i></div>
          <div class="ui-label split-head">Current · {driver.power.totalAmps.toFixed(1)} A</div>
          <div class="split">
            {#each driver.power.consumers as c, i (c.name)}
              <i style="flex:{Math.max(c.share, 0.02)}" class="seg s{i % 5}" title="{c.name} {c.amps.toFixed(1)} A"></i>
            {/each}
          </div>
          <ul class="consumers">
            {#each driver.power.consumers as c, i (c.name)}
              <li><i class="seg s{i % 5}"></i><span>{c.name}</span><b class="ui-mono">{c.amps.toFixed(1)} A</b></li>
            {/each}
          </ul>
        </div>
      </section>

      <section class="ui-panel tile alerts">
        <header class="ui-panel-head">
          <h3 class="ui-panel-title">Alerts</h3>
          <span class="ui-spacer"></span>
          {#if driver.alerts.length === 0}<span class="ui-badge" data-tone="ok">All clear</span>{/if}
        </header>
        {#if driver.alerts.length === 0}
          <p class="quiet">No faults or active stages.</p>
        {:else}
          <ul class="alert-list">
            {#each driver.alerts as alert (alert.key)}
              <li><span class="ui-dot" data-tone={alert.level}></span>{alert.text}</li>
            {/each}
          </ul>
        {/if}
      </section>

      <section class="ui-panel tile mechs">
        <header class="ui-panel-head"><h3 class="ui-panel-title">Mechanisms</h3></header>
        <div class="mech-grid">
          {#each mechanisms as m (m.node.path)}
            <button type="button" class="mech" onclick={() => onmechanism(m.node.path)}>
              <span class="ui-dot" data-tone={m.reason ? reasonTone(m.reason) : 'stale'}></span>
              <span class="mname">{m.node.name}</span>
              <span class="mreason">{m.reason || '—'}</span>
            </button>
          {/each}
        </div>
      </section>
    </div>
  {/if}
</div>

<style>
  .screen {
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: var(--sp-3);
    height: 100%;
    min-height: 0;
  }
  .strip {
    display: grid;
    gap: var(--sp-2);
    border-bottom: 1px solid var(--border);
    min-width: 0;
  }
  .tabs {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    gap: 2px;
    min-width: 0;
  }
  .tabs [role='tab'] {
    height: 34px;
    padding: 0 var(--sp-3);
    border: none;
    border-bottom: 2px solid transparent;
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-sm);
    font-weight: 500;
    cursor: pointer;
    margin-bottom: -1px;
    max-width: 220px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .tabs [role='tab']:hover {
    color: var(--text);
  }
  .tabs [role='tab'].active {
    color: var(--text-strong);
    border-bottom-color: var(--brand);
  }
  .tabs .new {
    margin-left: var(--sp-2);
  }
  .tabs .ui-spacer {
    flex: 1 1 auto;
  }
  .inline,
  .rename {
    display: inline-flex;
    align-items: center;
    flex-wrap: wrap;
    gap: var(--sp-2);
    margin-left: var(--sp-2);
  }
  .rename {
    margin: 0;
    padding: var(--sp-2) 0;
  }
  .inline .ui-input,
  .rename .ui-input {
    width: 200px;
  }
  .menu-wrap {
    position: relative;
    margin-left: var(--sp-2);
  }
  .menu {
    position: absolute;
    right: 0;
    top: calc(100% + 4px);
    z-index: 20;
    min-width: 180px;
    display: grid;
    padding: var(--sp-1);
    background: var(--surface-2);
    border: 1px solid var(--border-emphasis);
    border-radius: var(--radius-lg);
    box-shadow: 0 12px 28px rgba(0, 0, 0, 0.4);
  }
  .menu button {
    height: 30px;
    padding: 0 var(--sp-3);
    border: none;
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: var(--fs-sm);
    text-align: left;
    border-radius: var(--radius);
    cursor: pointer;
  }
  .menu button:hover {
    background: var(--surface-hover);
    color: var(--text-strong);
  }
  .menu button.danger {
    color: var(--fault);
  }
  .message {
    margin: 0 0 var(--sp-2);
    font-size: var(--fs-sm);
    color: var(--ok);
  }
  .message[data-tone='fault'] {
    color: var(--fault);
  }
  .canvas-host {
    display: flex;
    flex-direction: column;
    min-height: 0;
    min-width: 0;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(12, minmax(0, 1fr));
    grid-auto-rows: min-content;
    gap: var(--sp-3);
    overflow: auto;
    align-content: start;
  }
  .tile {
    grid-column: span 4;
  }
  .mechs {
    grid-column: span 12;
  }
  .match-body {
    display: grid;
    justify-items: start;
    gap: 6px;
  }
  .clock {
    font-variant-numeric: tabular-nums;
    font-size: 44px;
    font-weight: 600;
    line-height: 1;
    color: var(--text-strong);
    letter-spacing: -0.02em;
  }
  .phase {
    color: var(--text-soft);
    font-size: var(--fs-sm);
  }
  .battery {
    display: grid;
    gap: var(--sp-2);
  }
  .big {
    font-size: 32px;
    font-weight: 600;
    color: var(--text-strong);
    line-height: 1;
  }
  .big span {
    margin-left: 4px;
    font-size: var(--fs-md);
    color: var(--text-faint);
  }
  .big[data-tone='warn'] { color: var(--warn); }
  .big[data-tone='fault'] { color: var(--fault); }
  .meter {
    height: 6px;
    border-radius: 3px;
    background: var(--surface-3);
    overflow: hidden;
  }
  .meter i {
    display: block;
    height: 100%;
    background: var(--ok);
  }
  .meter i[data-tone='warn'] { background: var(--warn); }
  .meter i[data-tone='fault'] { background: var(--fault); }
  .split-head {
    margin-top: var(--sp-2);
  }
  .split {
    display: flex;
    gap: 2px;
    height: 8px;
    border-radius: 3px;
    overflow: hidden;
  }
  .seg {
    display: block;
  }
  .s0 { background: var(--series-reference); }
  .s1 { background: var(--series-measured); }
  .s2 { background: var(--series-target); }
  .s3 { background: var(--series-4); }
  .s4 { background: var(--series-5); }
  .consumers {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 4px;
    font-size: var(--fs-sm);
  }
  .consumers li {
    display: grid;
    grid-template-columns: 8px minmax(0, 1fr) auto;
    align-items: center;
    gap: var(--sp-2);
    color: var(--text-soft);
  }
  .consumers li i {
    width: 8px;
    height: 8px;
    border-radius: 2px;
  }
  .consumers b {
    color: var(--text-strong);
    font-weight: 500;
  }
  .quiet {
    margin: 0;
    padding: var(--sp-3);
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
  .alert-list {
    list-style: none;
    margin: 0;
    padding: var(--sp-2) var(--sp-3);
    display: grid;
    gap: var(--sp-2);
    font-size: var(--fs-sm);
  }
  .alert-list li {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
  }
  .mech-grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(220px, 1fr));
  }
  .mech {
    display: grid;
    grid-template-columns: 8px minmax(0, 1fr);
    grid-template-rows: auto auto;
    column-gap: var(--sp-2);
    align-items: center;
    padding: var(--sp-3);
    border: none;
    border-right: 1px solid var(--border);
    border-bottom: 1px solid var(--border);
    background: transparent;
    color: inherit;
    font: inherit;
    text-align: left;
    cursor: pointer;
  }
  .mech:hover {
    background: var(--surface-2);
  }
  .mname {
    font-weight: 600;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .mreason {
    grid-column: 2;
    font-size: var(--fs-xs);
    color: var(--text-faint);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  @media (max-width: 1100px) {
    .tile {
      grid-column: span 6;
    }
  }
  @media (max-width: 720px) {
    .tile {
      grid-column: span 12;
    }
  }
</style>
