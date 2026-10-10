<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { logLines } from '../../../lib/board/widgets/match-log';

  let { ctx }: { ctx: WidgetContext } = $props();

  const signal = $derived(ctx.bindings.text?.[0]);
  const capacity = $derived(ctx.setting<number>('lines'));
  const showTime = $derived(ctx.setting<boolean>('showTime'));

  // Hides lines logged before the operator pressed Clear (local only).
  let clearedAt = $state<number | null>(null);

  const lines = $derived.by(() => {
    void ctx.live.current;
    if (!signal) return [];
    return logLines(ctx.samples(signal), capacity, clearedAt);
  });

  function clock(micros: number): string {
    const total = micros / 1_000_000;
    const m = Math.floor(total / 60);
    const s = total - m * 60;
    return `${m}:${s.toFixed(1).padStart(4, '0')}`;
  }
</script>

<div class="log">
  <div class="bar">
    <span class="count ui-mono">{lines.length} {lines.length === 1 ? 'line' : 'lines'}</span>
    <span class="ui-spacer"></span>
    <button
      type="button"
      class="clear"
      disabled={ctx.editing || lines.length === 0}
      onclick={() => (clearedAt = lines[0]?.timestampMicros ?? null)}>Clear</button
    >
  </div>
  {#if lines.length === 0}
    <div class="empty">{signal ? 'Nothing logged yet' : 'Bind a text signal to log'}</div>
  {:else}
    <ol>
      {#each lines as line (line.timestampMicros)}
        <li>
          {#if showTime}<time class="ui-mono">{clock(line.timestampMicros)}</time>{/if}
          <span class="text">{line.text}</span>
        </li>
      {/each}
    </ol>
  {/if}
</div>

<style>
  .log {
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
    color: var(--text-soft);
  }
  .clear {
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
  .clear:disabled {
    opacity: 0.45;
    cursor: default;
  }
  ol {
    flex: 1;
    min-height: 0;
    margin: 0;
    padding: 0;
    list-style: none;
    overflow: auto;
  }
  li {
    display: grid;
    grid-template-columns: auto minmax(0, 1fr);
    gap: 10px;
    padding: 4px 8px;
    border-bottom: 1px solid var(--border);
    font-size: var(--fs-md);
    color: var(--text);
  }
  time {
    color: var(--text-faint);
    font-size: var(--fs-xs);
    padding-top: 2px;
  }
  .text {
    overflow-wrap: anywhere;
  }
  .empty {
    padding: 12px;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
