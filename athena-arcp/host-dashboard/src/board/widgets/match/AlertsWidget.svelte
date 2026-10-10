<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faXmark } from '@fortawesome/free-solid-svg-icons';
  import type { WidgetContext } from '../../../lib/board/types';
  import { MARKER_KIND } from '../../../lib/data/types';
  import { alertAge, buildAlerts, type Alert, type FaultSignalInput } from '../../../lib/board/widgets/match-alerts';
  import { robotNowMicros } from '../../../lib/driver/driver-state';

  let { ctx }: { ctx: WidgetContext } = $props();

  const faultSignals = $derived(ctx.bindings.faults ?? []);
  const limit = $derived(ctx.setting<number>('count'));
  const showWarnings = $derived(ctx.setting<boolean>('showWarnings'));

  /** Local dismissals (not shared with other viewers). */
  let dismissed = $state<Set<string>>(new Set());

  const feed = $derived.by(() => {
    void ctx.live.current;
    const store = ctx.session.store;
    const markers = store.markers().filter((m) => m.kind === MARKER_KIND.FAULT || m.kind === MARKER_KIND.STAGE_CHANGE || m.kind === MARKER_KIND.CHECK);
    const signals: FaultSignalInput[] = faultSignals.map((s) => {
      const sample = store.latestSample(s.signalId);
      return { path: s.path, active: sample?.value === true, timestampMicros: sample?.timestampMicros ?? null, name: s.name };
    });
    return buildAlerts({ markers, faultSignals: signals, dismissed, limit, showWarnings });
  });

  // Forget dismissals whose cause has gone, so a returning fault shows again.
  $effect(() => {
    const next = feed.dismissedNext;
    if (next.size !== dismissed.size) dismissed = next;
  });

  const now = $derived.by(() => {
    void ctx.live.current;
    return robotNowMicros(ctx.session.store);
  });

  function dismiss(alert: Alert) {
    dismissed = new Set([...dismissed, alert.key]);
  }

  function clearAll() {
    dismissed = new Set([...dismissed, ...feed.alerts.map((a) => a.key)]);
  }

</script>

<div class="alerts">
  <div class="bar">
    <span class="count ui-mono">{feed.alerts.length} {feed.alerts.length === 1 ? 'alert' : 'alerts'}</span>
    <span class="ui-spacer"></span>
    {#if feed.alerts.length > 0}
      <button type="button" class="clear" onclick={clearAll} disabled={ctx.editing}>Clear all</button>
    {/if}
  </div>
  {#if feed.alerts.length === 0}
    <div class="empty">All clear</div>
  {:else}
    <ul>
      {#each feed.alerts as alert (alert.key)}
        <li data-level={alert.level}>
          <span class="dot" aria-hidden="true"></span>
          <span class="text">{alert.text}</span>
          <span class="age ui-mono">{alertAge(alert.timestampMicros, now)}</span>
          <button type="button" class="dismiss" title="Dismiss" aria-label="Dismiss {alert.text}" onclick={() => dismiss(alert)} disabled={ctx.editing}>
            <FontAwesomeIcon icon={faXmark} />
          </button>
        </li>
      {/each}
    </ul>
  {/if}
</div>

<style>
  .alerts {
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
  .empty {
    flex: 1;
    display: grid;
    place-items: center;
    font-size: min(14cqh, 9cqw);
    font-weight: 600;
    color: var(--text-soft);
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
    display: grid;
    grid-template-columns: auto minmax(0, 1fr) auto auto;
    align-items: center;
    gap: 8px;
    min-height: 34px;
    padding: 4px 8px;
    border-bottom: 1px solid var(--border);
    font-size: min(max(12px, 4.2cqh), 15px);
    color: var(--text-strong);
  }
  li[data-level='fault'] {
    background: var(--fault-soft);
  }
  li[data-level='warn'] {
    background: var(--warn-soft);
  }
  .dot {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--stale);
  }
  li[data-level='fault'] .dot {
    background: var(--fault);
  }
  li[data-level='warn'] .dot {
    background: var(--warn);
  }
  li[data-level='info'] .dot {
    background: var(--info);
  }
  .text {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-weight: 600;
  }
  .age {
    font-size: var(--fs-xs);
    color: var(--text-soft);
    white-space: nowrap;
  }
  .dismiss {
    display: grid;
    place-items: center;
    width: 26px;
    height: 26px;
    border: none;
    border-radius: var(--radius-sm);
    background: transparent;
    color: var(--text-soft);
    cursor: pointer;
  }
  .dismiss:hover {
    background: var(--surface-3);
    color: var(--text-strong);
  }
  .dismiss:disabled {
    opacity: 0.3;
    cursor: default;
  }
</style>
