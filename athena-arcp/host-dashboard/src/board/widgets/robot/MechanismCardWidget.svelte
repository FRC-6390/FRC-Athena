<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { mechanismSiblings } from '../../../lib/board/widgets/robot-siblings';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { reasonTone, type Tone } from '../../../screens/mechanisms/tones';

  let { ctx }: { ctx: WidgetContext } = $props();

  const anchor = $derived(ctx.bindings.signal?.[0]);

  const sib = $derived.by(() => {
    void ctx.live.current;
    if (!anchor || anchor.mechanismPath === '') return null;
    return mechanismSiblings(ctx.session.store.allSignals(), anchor.mechanismPath);
  });

  const text = (s: SignalDescriptor | null | undefined): string => {
    void ctx.live.current;
    if (!s) return '';
    const v = ctx.latest(s);
    return typeof v === 'string' ? v : '';
  };
  const state = $derived(sib ? text(sib.state) : '');
  const reason = $derived(sib ? text(sib.reason) : '');
  const stage = $derived(sib ? text(sib.stage) : '');

  /** Fault labels: boolean fault flags that are set, and non-empty fault text (Athena's `State/Fault`). */
  const faulted = $derived.by((): { key: number; label: string }[] => {
    void ctx.live.current;
    if (!sib) return [];
    return sib.faults.flatMap((s) => {
      const v = ctx.latest(s);
      if (v === true) return [{ key: s.signalId, label: s.name }];
      if (typeof v === 'string' && v.trim() !== '') return [{ key: s.signalId, label: v }];
      return [];
    });
  });
  const activeActions = $derived.by(() => {
    void ctx.live.current;
    if (!sib) return [];
    return sib.actions.filter((s) => ctx.latest(s) === true);
  });

  const badge = $derived(faulted.length > 0 ? 'Fault' : state ? 'Running' : 'Idle');
  /** The reason without what the badge already says (`Running: WithinTolerance` -> `WithinTolerance`). */
  const reasonDetail = $derived.by(() => {
    const prefix = new RegExp(`^${badge}\\s*:?\\s*`, 'i');
    return reason.replace(prefix, '');
  });

  const tone: Tone = $derived.by(() => {
    if (faulted.length > 0) return 'fault';
    if (state) return 'info';
    if (reason) return reasonTone(reason);
    return 'stale';
  });
</script>

{#if !anchor}
  <div class="problem">Bind any signal from a mechanism</div>
{:else if !sib}
  <div class="problem">{anchor.path} is not part of a mechanism</div>
{:else}
  <div class="card">
    <header>
      <span class="ui-badge" data-tone={tone}>{badge}</span>
      <span class="reason-text" title={reason}>{reasonDetail}</span>
    </header>

    <dl>
      <dt>Action</dt>
      <dd class="ui-mono">{state || (activeActions.length > 0 ? activeActions.map((s) => s.name).join(', ') : 'none')}</dd>
      {#if sib.stage}
        <dt>Stage</dt>
        <dd class="ui-mono">{stage || '--'}</dd>
      {/if}
      <dt>Faults</dt>
      <dd>
        {#if faulted.length > 0}
          {#each faulted as f (f.key)}<span class="ui-badge" data-tone="fault" title={f.label}>{f.label}</span>{/each}
        {:else}
          <span class="none">none</span>
        {/if}
      </dd>
    </dl>
  </div>
{/if}

<style>
  .card {
    display: flex;
    flex-direction: column;
    gap: 10px;
    height: 100%;
    padding: 10px 12px;
    box-sizing: border-box;
    overflow: hidden;
  }
  header {
    display: flex;
    align-items: center;
    gap: 8px;
    min-width: 0;
  }
  .reason-text {
    flex: 1;
    min-width: 0;
    color: var(--text);
    font-size: var(--fs-sm);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  dl {
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 6px 12px;
    margin: 0;
    font-size: var(--fs-sm);
    align-items: center;
  }
  dt {
    color: var(--text-faint);
    font-size: var(--fs-xs);
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  dd {
    margin: 0;
    min-width: 0;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .none {
    color: var(--text-faint);
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
