<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { parseItems } from '../../../lib/board/widgets/match-layout';
  import { checklistKey, loadChecked, saveChecked } from '../../../lib/board/widgets/match-checklist';

  let { ctx }: { ctx: WidgetContext } = $props();

  const items = $derived(parseItems(ctx.setting<string>('items')));
  const storageKey = $derived(checklistKey(ctx.setting<string>('storageKey'), items));

  let checked = $state<string[]>([]);
  let loadedKey = $state<string | null>(null);

  // Load ticks when the list identity changes; never write during the load.
  $effect(() => {
    const key = storageKey;
    if (key === loadedKey) return;
    checked = loadChecked(key);
    loadedKey = key;
  });

  const done = $derived(items.filter((item) => checked.includes(item)).length);
  const complete = $derived(items.length > 0 && done === items.length);

  function toggle(item: string) {
    if (ctx.editing) return;
    checked = checked.includes(item) ? checked.filter((c) => c !== item) : [...checked, item];
    saveChecked(storageKey, checked.filter((c) => items.includes(c)));
  }

  function reset() {
    checked = [];
    saveChecked(storageKey, []);
  }
</script>

<div class="list" data-complete={complete}>
  <div class="bar">
    <span class="progress ui-mono">{done} / {items.length}</span>
    <span class="ui-spacer"></span>
    <button type="button" class="reset" onclick={reset} disabled={ctx.editing || done === 0}>Reset</button>
  </div>
  {#if items.length === 0}
    <div class="empty">No items. Add them in settings, separated by |.</div>
  {:else}
    <ul>
      {#each items as item (item)}
        <li>
          <label class:done={checked.includes(item)}>
            <input type="checkbox" checked={checked.includes(item)} disabled={ctx.editing} onchange={() => toggle(item)} />
            <span>{item}</span>
          </label>
        </li>
      {/each}
    </ul>
  {/if}
</div>

<style>
  .list {
    display: flex;
    flex-direction: column;
    height: 100%;
    min-height: 0;
    container-type: size;
  }
  .bar {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 4px 8px;
    border-bottom: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  .progress {
    color: var(--text-soft);
  }
  .list[data-complete='true'] .progress {
    color: var(--ok);
    font-weight: 700;
  }
  .reset {
    height: 24px;
    padding: 0 8px;
    border: 1px solid var(--border-subtle);
    border-radius: var(--radius-sm);
    background: var(--surface-3);
    color: var(--text);
    font: inherit;
    font-size: var(--fs-xs);
    cursor: pointer;
  }
  .reset:disabled {
    opacity: 0.45;
    cursor: default;
  }
  ul {
    flex: 1;
    min-height: 0;
    margin: 0;
    padding: 0;
    list-style: none;
    overflow: auto;
  }
  li {
    border-bottom: 1px solid var(--border);
  }
  label {
    display: flex;
    align-items: center;
    gap: 10px;
    min-height: 36px;
    padding: 4px 10px;
    cursor: pointer;
    font-size: min(max(14px, 4.4cqh), 22px);
    color: var(--text-strong);
  }
  label.done {
    color: var(--text-faint);
    text-decoration: line-through;
  }
  input {
    width: 20px;
    height: 20px;
    margin: 0;
    flex: none;
    accent-color: var(--ok);
    cursor: pointer;
  }
  input:disabled {
    cursor: default;
  }
  .empty {
    padding: 12px;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
