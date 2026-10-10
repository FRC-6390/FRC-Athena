<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faXmark } from '@fortawesome/free-solid-svg-icons';
  import type { Session } from '../lib/data/session';
  import type { WidgetInstance, SettingField } from '../lib/board/types';
  import { matches, widgetDef } from '../lib/board/registry';

  type Props = {
    instance: WidgetInstance;
    session: Session;
    live: { readonly current: number };
    onchange: (next: WidgetInstance) => void;
    onclose: () => void;
    onremove: () => void;
  };

  let { instance, session, live, onchange, onclose, onremove }: Props = $props();

  const def = $derived(widgetDef(instance.type));
  let search = $state<Record<string, string>>({});

  function update(mutate: (draft: WidgetInstance) => void) {
    const next = structuredClone($state.snapshot(instance)) as WidgetInstance;
    mutate(next);
    onchange(next);
  }

  function candidates(slotKey: string): string[] {
    void live.current;
    const slot = def?.slots.find((s) => s.key === slotKey);
    if (!slot) return [];
    const needle = (search[slotKey] ?? '').trim().toLowerCase();
    const bound = new Set(instance.bindings[slotKey] ?? []);
    return session.store
      .allSignals()
      .filter((s) => matches(slot.matcher, s) && !bound.has(s.path) && (!needle || s.path.toLowerCase().includes(needle)))
      .map((s) => s.path)
      .sort()
      .slice(0, 30);
  }

  function setBinding(slotKey: string, path: string, multiple: boolean) {
    update((d) => {
      const list = d.bindings[slotKey] ?? [];
      d.bindings[slotKey] = multiple ? [...list, path] : [path];
    });
    search[slotKey] = '';
  }

  function settingValue(field: SettingField): unknown {
    return field.key in instance.settings ? instance.settings[field.key] : field.default;
  }

  function setSetting(field: SettingField, raw: string | boolean) {
    update((d) => {
      if (field.kind === 'number') {
        d.settings[field.key] = raw === '' ? Number.NaN : Number(raw);
      } else {
        d.settings[field.key] = raw;
      }
    });
  }

  const numberText = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? String(v) : '');
</script>

<aside class="drawer ui-panel">
  <header class="ui-panel-head">
    <h3 class="ui-panel-title">{def?.name ?? instance.type}</h3>
    <span class="ui-spacer"></span>
    <button type="button" class="ui-btn" data-variant="ghost" data-size="sm" title="Close" onclick={onclose}><FontAwesomeIcon icon={faXmark} /></button>
  </header>
  <div class="scroll">
    <section>
      <label class="field">
        <span class="ui-label">Title</span>
        <input class="ui-input text" placeholder="Automatic" value={instance.title ?? ''} oninput={(e) => update((d) => (d.title = e.currentTarget.value))} />
      </label>
      <label class="check">
        <input type="checkbox" checked={instance.settings.showTitle !== false} onchange={(e) => update((d) => (d.settings.showTitle = e.currentTarget.checked))} />
        Show title bar
      </label>
    </section>

    {#if def}
      {#each def.slots as slot (slot.key)}
        <section>
          <div class="ui-label">{slot.label}{slot.multiple ? ' (several)' : ''}</div>
          {#each instance.bindings[slot.key] ?? [] as path (path)}
            <div class="bound">
              <span class="ui-mono" title={path}>{path.replace(/^robot\//, '')}</span>
              <button type="button" title="Remove" onclick={() => update((d) => (d.bindings[slot.key] = (d.bindings[slot.key] ?? []).filter((p) => p !== path)))}>
                <FontAwesomeIcon icon={faXmark} />
              </button>
            </div>
          {/each}
          {#if slot.multiple || (instance.bindings[slot.key]?.length ?? 0) === 0}
            <input class="ui-input text" placeholder="Search to add a signal" bind:value={search[slot.key]} />
            {#if (search[slot.key] ?? '') !== ''}
              <div class="results">
                {#each candidates(slot.key) as path (path)}
                  <button type="button" onclick={() => setBinding(slot.key, path, slot.multiple === true)}>{path.replace(/^robot\//, '')}</button>
                {:else}
                  <span class="none">No matching signals</span>
                {/each}
              </div>
            {/if}
          {/if}
        </section>
      {/each}

      {#if def.settings.length > 0}
        <section>
          <div class="ui-label">Options</div>
          {#each def.settings as field (field.key)}
            {@const value = settingValue(field)}
            {#if field.kind === 'bool'}
              <label class="check">
                <input type="checkbox" checked={value === true} onchange={(e) => setSetting(field, e.currentTarget.checked)} />
                {field.label}
              </label>
            {:else}
              <label class="field">
                <span class="flabel">{field.label}{field.kind === 'number' && field.unit ? ` (${field.unit})` : ''}</span>
                {#if field.kind === 'enum'}
                  <select class="ui-input" value={String(value)} onchange={(e) => setSetting(field, e.currentTarget.value)}>
                    {#each field.options as option (option.value)}<option value={option.value}>{option.label}</option>{/each}
                  </select>
                {:else if field.kind === 'number'}
                  <input class="ui-input" type="number" step={field.step ?? 'any'} min={field.min} max={field.max} value={numberText(value)} onchange={(e) => setSetting(field, e.currentTarget.value)} />
                {:else if field.kind === 'textarea'}
                  <textarea class="ui-input area" rows={field.rows ?? 5} placeholder={field.placeholder} value={String(value ?? '')} oninput={(e) => setSetting(field, e.currentTarget.value)}></textarea>
                {:else if field.kind === 'color'}
                  <input class="ui-input" type="color" value={String(value)} onchange={(e) => setSetting(field, e.currentTarget.value)} />
                {:else}
                  <input class="ui-input text" value={String(value ?? '')} oninput={(e) => setSetting(field, e.currentTarget.value)} />
                {/if}
              </label>
            {/if}
          {/each}
        </section>
      {/if}
    {/if}

    <section class="footer">
      <span class="size ui-mono">{instance.w} × {instance.h} cells</span>
      <button type="button" class="ui-btn danger" data-size="sm" onclick={onremove}>Remove widget</button>
    </section>
  </div>
</aside>

<style>
  .drawer {
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .scroll {
    overflow: auto;
    min-height: 0;
  }
  section {
    display: grid;
    gap: 6px;
    padding: var(--sp-3);
    border-bottom: 1px solid var(--border);
  }
  .field {
    display: grid;
    gap: 4px;
  }
  .flabel {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .text {
    font-family: var(--font-body);
  }
  .area {
    height: auto;
    padding: 6px 8px;
    font-family: var(--font-body);
    line-height: 1.4;
    resize: vertical;
  }
  .check {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: var(--fs-sm);
    color: var(--text);
  }
  .bound {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 4px 4px 4px 8px;
    border-radius: var(--radius-sm);
    background: var(--surface-2);
    font-size: var(--fs-xs);
  }
  .bound span {
    flex: 1;
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .bound button {
    border: none;
    background: transparent;
    color: var(--text-faint);
    cursor: pointer;
  }
  .bound button:hover {
    color: var(--fault);
  }
  .results {
    display: grid;
    max-height: 180px;
    overflow: auto;
    border: 1px solid var(--border);
    border-radius: var(--radius-sm);
  }
  .results button {
    padding: 5px 8px;
    border: none;
    background: transparent;
    color: var(--text);
    font-family: var(--font-mono);
    font-size: var(--fs-xs);
    text-align: left;
    cursor: pointer;
  }
  .results button:hover {
    background: var(--surface-3);
  }
  .none {
    padding: 6px 8px;
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .footer {
    display: flex;
    align-items: center;
    justify-content: space-between;
    border-bottom: none;
  }
  .size {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .danger:hover {
    border-color: var(--fault);
    color: var(--fault);
  }
</style>
