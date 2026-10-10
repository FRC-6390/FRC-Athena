<script lang="ts">
  import { onMount } from 'svelte';
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import {
    faPlus,
    faPen,
    faCheck,
    faRotateLeft,
    faRotateRight,
    faEllipsis,
    faLock,
    faLockOpen
  } from '@fortawesome/free-solid-svg-icons';
  import type { Session } from '../lib/data/session';
  import type { DriverState } from '../lib/driver/driver-state';
  import type { BoardDoc, WidgetDef, WidgetInstance } from '../lib/board/types';
  import { createWidget, newId, suggestWidgets, widgetDef } from '../lib/board/registry';
  import { exportBoards, importBoards, loadBoards, saveBoards } from '../lib/board/storage';
  import { History } from '../lib/board/history';
  import { driverBoard, tunableMechanisms, tuningBoard } from '../lib/board/templates';
  import { downloadJsonFile, pickJsonFileText, shouldIgnoreGlobalHotkeys } from '../lib/browser-io';
  import { registerAllWidgets } from '../board/widgets';
  import Board, { type DragPayload } from '../board/Board.svelte';
  import SignalDrawer from '../board/SignalDrawer.svelte';
  import SettingsDrawer from '../board/SettingsDrawer.svelte';
  import WidgetPicker from '../board/WidgetPicker.svelte';
  import OverviewBoard from './OverviewBoard.svelte';

  registerAllWidgets();

  type Props = {
    session: Session;
    live: { readonly current: number };
    driver: DriverState;
    onmechanism: (path: string) => void;
  };

  let { session, live, driver, onmechanism }: Props = $props();

  const OVERVIEW = '__overview__';
  let boards = $state<BoardDoc[]>(loadBoards());
  let activeId = $state<string>(OVERVIEW);
  let editing = $state(false);
  let selectedId = $state<string | null>(null);
  let menuOpen = $state(false);
  let newOpen = $state(false);
  let renaming = $state(false);
  let nameDraft = $state('');
  let notice = $state<string | null>(null);
  let picker = $state<{ path: string; options: WidgetDef[]; at: { left: number; top: number }; cell: { x: number; y: number } } | null>(null);
  const history = new History<BoardDoc[]>();
  let historyTick = $state(0);
  let lastPointer = { left: 0, top: 0 };

  const active = $derived(boards.find((b) => b.id === activeId) ?? null);
  const selected = $derived(active?.widgets.find((w) => w.id === selectedId) ?? null);

  function say(text: string) {
    notice = text;
    setTimeout(() => {
      if (notice === text) notice = null;
    }, 3000);
  }

  /** Every edit goes through here: snapshot for undo, apply, persist. */
  function change(mutate: (draft: BoardDoc[]) => void) {
    const before = $state.snapshot(boards) as BoardDoc[];
    history.push(before);
    const next = structuredClone(before);
    mutate(next);
    boards = next;
    saveBoards(next);
    historyTick++;
  }

  function changeActive(mutate: (board: BoardDoc) => void) {
    change((all) => {
      const board = all.find((b) => b.id === activeId);
      if (board) mutate(board);
    });
  }

  function restore(state: BoardDoc[] | null) {
    if (!state) return;
    if (!state.some((b) => b.id === activeId)) activeId = state.at(-1)?.id ?? OVERVIEW;
    boards = state;
    saveBoards(state);
    historyTick++;
  }
  const undo = () => restore(history.undo($state.snapshot(boards) as BoardDoc[]));
  const redo = () => restore(history.redo($state.snapshot(boards) as BoardDoc[]));

  function addBoard(board: BoardDoc, edit: boolean) {
    change((all) => {
      all.push(board);
    });
    activeId = board.id;
    selectedId = null;
    editing = edit;
    newOpen = false;
  }

  function newBoard() {
    addBoard({ id: newId('b'), name: `Board ${boards.length + 1}`, columns: 24, widgets: [] }, true);
    renaming = true;
    nameDraft = `Board ${boards.length}`;
  }

  const tunable = $derived.by(() => {
    void live.current;
    return newOpen ? tunableMechanisms(session.store.allSignals()) : [];
  });

  function nextFreeRow(board: BoardDoc): number {
    return board.widgets.reduce((max, w) => Math.max(max, w.y + w.h), 0);
  }

  function addWidget(def: WidgetDef, cell: { x: number; y: number }, path?: string) {
    const signal = path ? session.store.descriptorByPath(path) : undefined;
    const instance = createWidget(def, cell, signal);
    instance.x = Math.min(cell.x, Math.max(0, (active?.columns ?? 24) - instance.w));
    changeActive((b) => {
      b.widgets.push(instance);
    });
    selectedId = instance.id;
  }

  function handleDrop(payload: DragPayload, cell: { x: number; y: number }, at?: { left: number; top: number }) {
    if (!active) return;
    if (payload.kind === 'widget') {
      const def = widgetDef(payload.type);
      if (def) addWidget(def, cell);
      return;
    }
    const signal = session.store.descriptorByPath(payload.path);
    if (!signal) return;
    const options = suggestWidgets(signal);
    if (options.length === 1) {
      addWidget(options[0], cell, payload.path);
      return;
    }
    picker = {
      path: payload.path,
      options,
      cell,
      at: at ?? { left: window.innerWidth / 2 - 140, top: window.innerHeight / 3 }
    };
  }

  function updateWidget(next: WidgetInstance) {
    changeActive((b) => {
      const i = b.widgets.findIndex((w) => w.id === next.id);
      if (i >= 0) b.widgets[i] = next;
    });
  }

  function removeWidget(id: string) {
    changeActive((b) => {
      b.widgets = b.widgets.filter((w) => w.id !== id);
    });
    if (selectedId === id) selectedId = null;
  }

  function duplicateWidget(id: string) {
    const source = active?.widgets.find((w) => w.id === id);
    if (!source || !active) return;
    const copy = { ...(structuredClone($state.snapshot(source)) as WidgetInstance), id: newId(), y: nextFreeRow(active) };
    changeActive((b) => {
      b.widgets.push(copy);
    });
    selectedId = copy.id;
  }

  function applyLayout(changes: Array<{ id: string; x: number; y: number; w: number; h: number }>) {
    const current = active?.widgets ?? [];
    const differs = changes.some((c) => {
      const w = current.find((x) => x.id === c.id);
      return w && (w.x !== c.x || w.y !== c.y || w.w !== c.w || w.h !== c.h);
    });
    if (!differs) return;
    changeActive((b) => {
      for (const c of changes) {
        const w = b.widgets.find((x) => x.id === c.id);
        if (w) Object.assign(w, { x: c.x, y: c.y, w: c.w, h: c.h });
      }
    });
  }

  function commitRename() {
    const name = nameDraft.trim();
    if (renaming && name && name !== active?.name) {
      changeActive((b) => {
        b.name = name;
      });
    }
    renaming = false;
  }

  function duplicateBoard() {
    if (!active) return;
    const copy = { ...(structuredClone($state.snapshot(active)) as BoardDoc), id: newId('b'), name: `${active.name} copy`, locked: false };
    change((all) => {
      all.push(copy);
    });
    activeId = copy.id;
    menuOpen = false;
  }

  function deleteBoard() {
    if (!active) return;
    const name = active.name;
    change((all) => {
      all.splice(all.findIndex((b) => b.id === activeId), 1);
    });
    activeId = OVERVIEW;
    editing = false;
    menuOpen = false;
    say(`Deleted “${name}” (Ctrl+Z to restore)`);
  }

  function exportActive() {
    if (!active) return;
    downloadJsonFile(`${active.name.replace(/[^\w-]+/g, '_')}.athena-board.json`, exportBoards([$state.snapshot(active) as BoardDoc]));
    menuOpen = false;
  }

  async function importFile() {
    menuOpen = false;
    const text = await pickJsonFileText();
    if (!text) return;
    try {
      const imported = importBoards(text).map((b) => ({ ...b, id: newId('b') }));
      change((all) => {
        all.push(...imported);
      });
      activeId = imported[0].id;
      say(`Imported ${imported.length} board${imported.length === 1 ? '' : 's'}`);
    } catch (error) {
      say(error instanceof Error ? error.message : String(error));
    }
  }

  function toggleLock() {
    const locking = !active?.locked;
    changeActive((b) => {
      b.locked = locking;
    });
    if (locking) editing = false;
    menuOpen = false;
  }

  onMount(() => {
    const onKey = (event: KeyboardEvent) => {
      if (shouldIgnoreGlobalHotkeys(event.target)) return;
      const mod = event.ctrlKey || event.metaKey;
      const key = event.key.toLowerCase();
      if (mod && key === 'z') {
        event.preventDefault();
        if (event.shiftKey) redo();
        else undo();
      } else if (mod && key === 'y') {
        event.preventDefault();
        redo();
      } else if ((event.key === 'Delete' || event.key === 'Backspace') && editing && selectedId) {
        event.preventDefault();
        removeWidget(selectedId);
      } else if (event.key === 'Escape') {
        selectedId = null;
      } else if (key === 'e' && !mod && active && !active.locked) {
        editing = !editing;
        selectedId = null;
      }
    };
    const onPointer = (event: PointerEvent | DragEvent) => {
      lastPointer = { left: event.clientX, top: event.clientY };
    };
    window.addEventListener('keydown', onKey);
    window.addEventListener('pointermove', onPointer);
    window.addEventListener('dragover', onPointer);
    return () => {
      window.removeEventListener('keydown', onKey);
      window.removeEventListener('pointermove', onPointer);
      window.removeEventListener('dragover', onPointer);
    };
  });
</script>

<div class="screen">
  <div class="bar">
    <div class="tabs" role="tablist">
      <button
        type="button"
        role="tab"
        aria-selected={activeId === OVERVIEW}
        class:active={activeId === OVERVIEW}
        onclick={() => {
          activeId = OVERVIEW;
          editing = false;
        }}>Overview</button
      >
      {#each boards as board (board.id)}
        <button
          type="button"
          role="tab"
          aria-selected={activeId === board.id}
          class:active={activeId === board.id}
          onclick={() => {
            if (activeId !== board.id) {
              activeId = board.id;
              selectedId = null;
              editing = false;
            }
          }}
          ondblclick={() => {
            if (!board.locked) {
              activeId = board.id;
              renaming = true;
              nameDraft = board.name;
            }
          }}
        >
          {#if board.locked}<FontAwesomeIcon icon={faLock} />{/if}
          {#if renaming && activeId === board.id}
            <!-- svelte-ignore a11y_autofocus -->
            <input
              class="rename"
              autofocus
              bind:value={nameDraft}
              onclick={(e) => e.stopPropagation()}
              onkeydown={(e) => {
                if (e.key === 'Enter') commitRename();
                if (e.key === 'Escape') renaming = false;
              }}
              onblur={commitRename}
            />
          {:else}
            {board.name}
          {/if}
        </button>
      {/each}
    </div>
    <div class="new-wrap">
      <button type="button" class="new" title="New board" onclick={() => (newOpen = !newOpen)}><FontAwesomeIcon icon={faPlus} /></button>
      {#if newOpen}
        <div class="scrim" role="presentation" onpointerdown={() => (newOpen = false)}></div>
        <div class="menu new-menu ui-panel" role="menu">
          <button type="button" role="menuitem" onclick={newBoard}>
            <span class="mi-title">Blank board</span><span class="mi-desc">Start empty and drag signals in</span>
          </button>
          <button type="button" role="menuitem" onclick={() => addBoard(driverBoard(session.store.allSignals()), false)}>
            <span class="mi-title">Driver</span><span class="mi-desc">Match clock, battery, alerts, field, camera, mechanism status</span>
          </button>
          {#if tunable.length > 0}
            <hr />
            <div class="mi-head">Tune a mechanism</div>
            {#each tunable as mechanism (mechanism.path)}
              <button
                type="button"
                role="menuitem"
                onclick={() => {
                  const board = tuningBoard(session.store.allSignals(), mechanism.path);
                  if (board) addBoard(board, false);
                }}
              >
                <span class="mi-title">{mechanism.name}</span>
              </button>
            {/each}
          {:else}
            <p class="mi-note">Connect to a robot for driver and tuning templates.</p>
          {/if}
        </div>
      {/if}
    </div>

    <span class="ui-spacer"></span>
    {#if notice}<span class="notice">{notice}</span>{/if}

    {#if active}
      {#if editing}
        {#key historyTick}
          <button type="button" class="ui-btn" data-variant="ghost" data-size="sm" title="Undo (Ctrl+Z)" disabled={!history.canUndo} onclick={undo}>
            <FontAwesomeIcon icon={faRotateLeft} />
          </button>
          <button type="button" class="ui-btn" data-variant="ghost" data-size="sm" title="Redo (Ctrl+Shift+Z)" disabled={!history.canRedo} onclick={redo}>
            <FontAwesomeIcon icon={faRotateRight} />
          </button>
        {/key}
      {/if}
      {#if !active.locked}
        <button
          type="button"
          class="ui-btn"
          data-variant={editing ? 'primary' : undefined}
          data-size="sm"
          title="Edit (E)"
          onclick={() => {
            editing = !editing;
            selectedId = null;
          }}
        >
          <FontAwesomeIcon icon={editing ? faCheck : faPen} />
          {editing ? 'Done' : 'Edit'}
        </button>
      {/if}
      <div class="menu-wrap">
        <button type="button" class="ui-btn" data-variant="ghost" data-size="sm" title="Board options" onclick={() => (menuOpen = !menuOpen)}>
          <FontAwesomeIcon icon={faEllipsis} />
        </button>
        {#if menuOpen}
          <div class="scrim" role="presentation" onpointerdown={() => (menuOpen = false)}></div>
          <div class="menu ui-panel" role="menu">
            <button
              type="button"
              role="menuitem"
              onclick={() => {
                renaming = true;
                nameDraft = active.name;
                menuOpen = false;
              }}>Rename</button
            >
            <button type="button" role="menuitem" onclick={duplicateBoard}>Duplicate</button>
            <button type="button" role="menuitem" onclick={toggleLock}>
              <FontAwesomeIcon icon={active.locked ? faLockOpen : faLock} />
              {active.locked ? 'Unlock board' : 'Lock for driving'}
            </button>
            <hr />
            <button type="button" role="menuitem" onclick={exportActive}>Export…</button>
            <button type="button" role="menuitem" onclick={importFile}>Import…</button>
            <hr />
            <button type="button" role="menuitem" class="danger" onclick={deleteBoard}>Delete board</button>
          </div>
        {/if}
      </div>
    {:else}
      <button type="button" class="ui-btn" data-size="sm" onclick={importFile}>Import…</button>
    {/if}
  </div>

  {#if activeId === OVERVIEW || !active}
    <OverviewBoard {session} {live} {driver} {onmechanism} />
  {:else}
    <div class="workspace" class:editing class:with-settings={editing && selected}>
      {#if editing}
        <SignalDrawer {session} {live} onadd={(payload) => active && handleDrop(payload, { x: 0, y: nextFreeRow(active) })} />
      {/if}
      <div class="board-scroll">
        {#key active.id}
          <Board
            board={active}
            {session}
            {live}
            {editing}
            {selectedId}
            onselect={(id) => (selectedId = id)}
            onlayout={applyLayout}
            onremove={removeWidget}
            onduplicate={duplicateWidget}
            ondrop={(payload, cell) => handleDrop(payload, cell, lastPointer)}
          />
        {/key}
      </div>
      {#if editing && selected}
        <SettingsDrawer
          instance={selected}
          {session}
          {live}
          onchange={updateWidget}
          onclose={() => (selectedId = null)}
          onremove={() => removeWidget(selected.id)}
        />
      {/if}
    </div>
  {/if}
</div>

{#if picker}
  {@const signal = session.store.descriptorByPath(picker.path)}
  {#if signal}
    <WidgetPicker
      {signal}
      options={picker.options}
      at={picker.at}
      onpick={(def) => {
        if (picker) addWidget(def, picker.cell, picker.path);
        picker = null;
      }}
      oncancel={() => (picker = null)}
    />
  {/if}
{/if}

<style>
  .screen {
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: var(--sp-3);
    height: 100%;
    min-height: 0;
  }
  .bar {
    display: flex;
    align-items: center;
    gap: var(--sp-2);
    border-bottom: 1px solid var(--border);
    min-width: 0;
  }
  .tabs {
    display: flex;
    align-items: center;
    gap: 2px;
    min-width: 0;
    overflow-x: auto;
  }
  .tabs [role='tab'] {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    height: 34px;
    padding: 0 var(--sp-3);
    border: none;
    border-bottom: 2px solid transparent;
    margin-bottom: -1px;
    background: transparent;
    color: var(--text-soft);
    font: inherit;
    font-size: var(--fs-sm);
    font-weight: 500;
    white-space: nowrap;
    cursor: pointer;
  }
  .tabs [role='tab']:hover {
    color: var(--text);
  }
  .tabs [role='tab'].active {
    color: var(--text-strong);
    border-bottom-color: var(--brand);
  }
  .tabs [role='tab'] :global(svg) {
    font-size: 10px;
    color: var(--locked);
  }
  .rename {
    width: 140px;
    height: 24px;
    border: 1px solid var(--info);
    border-radius: var(--radius-sm);
    background: var(--bg-1);
    color: var(--text-strong);
    font: inherit;
    padding: 0 6px;
  }
  .new-wrap {
    position: relative;
  }
  .new-menu {
    left: 0;
    right: auto;
    width: 300px;
    max-height: 60vh;
    overflow: auto;
  }
  .new-menu button {
    display: grid;
    gap: 1px;
  }
  .mi-title {
    font-weight: 600;
    color: var(--text-strong);
    text-transform: capitalize;
  }
  .mi-desc {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .mi-head {
    padding: 4px 10px;
    font-size: var(--fs-xs);
    font-weight: 600;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--text-faint);
  }
  .mi-note {
    margin: 0;
    padding: 6px 10px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .new {
    display: grid;
    place-items: center;
    width: 28px;
    height: 28px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text-faint);
    cursor: pointer;
  }
  .new:hover {
    background: var(--surface-3);
    color: var(--text);
  }
  .notice {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .menu-wrap {
    position: relative;
  }
  .scrim {
    position: fixed;
    inset: 0;
    z-index: 30;
  }
  .menu {
    position: absolute;
    right: 0;
    top: calc(100% + 4px);
    z-index: 31;
    display: grid;
    min-width: 190px;
    padding: 4px;
    box-shadow: 0 12px 32px rgba(0, 0, 0, 0.45);
  }
  .menu button {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 7px 10px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text);
    font: inherit;
    font-size: var(--fs-sm);
    text-align: left;
    cursor: pointer;
  }
  .menu button:hover {
    background: var(--surface-3);
  }
  .menu .danger {
    color: var(--fault);
  }
  .menu hr {
    width: 100%;
    margin: 4px 0;
    border: none;
    border-top: 1px solid var(--border);
  }
  .workspace {
    display: grid;
    grid-template-columns: minmax(0, 1fr);
    gap: var(--sp-3);
    min-height: 0;
  }
  .workspace.editing {
    grid-template-columns: 260px minmax(0, 1fr);
  }
  .workspace.editing.with-settings {
    grid-template-columns: 260px minmax(0, 1fr) 300px;
  }
  .board-scroll {
    min-height: 0;
    overflow: auto;
    padding-bottom: var(--sp-5);
  }
</style>
