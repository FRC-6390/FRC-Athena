<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faGripVertical, faClone, faTrash, faTriangleExclamation } from '@fortawesome/free-solid-svg-icons';
  import type { Session } from '../lib/data/session';
  import type { WidgetInstance } from '../lib/board/types';
  import { widgetDef } from '../lib/board/registry';
  import { createContext, resolveBindings, widgetTitle } from '../lib/board/context';

  type Props = {
    instance: WidgetInstance;
    session: Session;
    live: { readonly current: number };
    editing: boolean;
    selected: boolean;
    onselect: () => void;
    onremove: () => void;
    onduplicate: () => void;
  };

  let { instance, session, live, editing, selected, onselect, onremove, onduplicate }: Props = $props();

  let width = $state(0);
  let height = $state(0);

  const def = $derived(widgetDef(instance.type));
  const bindings = $derived.by(() => {
    void live.current;
    return resolveBindings(session, instance);
  });
  const missing = $derived(
    def
      ? def.slots.filter((s) => s.required && (bindings[s.key]?.length ?? 0) === 0).map((s) => s.label)
      : []
  );
  const ctx = $derived(
    def
      ? createContext({ session, live, instance, def, bindings, size: { width, height }, editing })
      : null
  );
  const title = $derived(def ? widgetTitle(instance, def, bindings) : instance.type);
  const showTitle = $derived(instance.settings.showTitle !== false);
</script>

<!-- svelte-ignore a11y_click_events_have_key_events -->
<!-- svelte-ignore a11y_no_static_element_interactions -->
<div class="frame" class:selected class:editing onclick={() => editing && onselect()}>
  {#if showTitle || editing}
    <header class="head">
      {#if editing}<span class="drag-handle" title="Drag to move"><FontAwesomeIcon icon={faGripVertical} /></span>{/if}
      <span class="title" title={title}>{title}</span>
      {#if editing}
        <span class="tools">
          <button type="button" title="Duplicate" onclick={(e) => { e.stopPropagation(); onduplicate(); }}><FontAwesomeIcon icon={faClone} /></button>
          <button type="button" title="Remove" onclick={(e) => { e.stopPropagation(); onremove(); }}><FontAwesomeIcon icon={faTrash} /></button>
        </span>
      {/if}
    </header>
  {/if}
  <div class="body" bind:clientWidth={width} bind:clientHeight={height}>
    {#if !def}
      <div class="problem"><FontAwesomeIcon icon={faTriangleExclamation} /> Unknown widget “{instance.type}”</div>
    {:else if missing.length > 0}
      <div class="problem">
        {editing ? `Bind ${missing.join(', ')} in settings` : `Waiting for ${Object.values(instance.bindings).flat()[0] ?? 'a signal'}`}
      </div>
    {:else if ctx}
      <def.component {ctx} />
    {/if}
  </div>
</div>

<style>
  .frame {
    display: flex;
    flex-direction: column;
    height: 100%;
    border: 1px solid var(--border);
    border-radius: var(--radius-lg);
    background: var(--surface);
    overflow: hidden;
  }
  .frame.editing {
    cursor: default;
  }
  .frame.editing:hover {
    border-color: var(--border-emphasis);
  }
  .frame.selected {
    border-color: var(--info);
    box-shadow: 0 0 0 1px var(--info);
  }
  .head {
    display: flex;
    align-items: center;
    gap: 6px;
    height: 28px;
    flex: none;
    padding: 0 8px;
    border-bottom: 1px solid var(--border);
  }
  .drag-handle {
    color: var(--text-faint);
    cursor: grab;
    font-size: 11px;
    padding: 4px 2px;
  }
  .drag-handle:active {
    cursor: grabbing;
  }
  .title {
    flex: 1;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: var(--fs-xs);
    font-weight: 600;
    letter-spacing: 0.05em;
    text-transform: uppercase;
    color: var(--text-soft);
  }
  .tools {
    display: inline-flex;
    gap: 2px;
  }
  .tools button {
    display: grid;
    place-items: center;
    width: 22px;
    height: 22px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text-faint);
    font-size: 11px;
    cursor: pointer;
  }
  .tools button:hover {
    background: var(--surface-3);
    color: var(--text);
  }
  .body {
    position: relative;
    flex: 1;
    min-height: 0;
    overflow: hidden;
  }
  .problem {
    display: grid;
    place-items: center;
    height: 100%;
    padding: 8px;
    text-align: center;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
