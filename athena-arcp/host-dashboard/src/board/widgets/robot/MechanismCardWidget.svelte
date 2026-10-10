<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { mechanismSiblings } from '../../../lib/board/widgets/robot-siblings';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { reasonTone, stateTone, type Tone } from '../../../screens/mechanisms/tones';

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

  const faulted = $derived.by(() => {
    void ctx.live.current;
    if (!sib) return [];
    return sib.faults.filter((s) => ctx.latest(s) === true);
  });
  const activeActions = $derived.by(() => {
    void ctx.live.current;
    if (!sib) return [];
    return sib.actions.filter((s) => ctx.latest(s) === true);
  });

  const tone: Tone = $derived.by(() => {
    if (faulted.length > 0) return 'fault';
    if (reason) return reasonTone(reason);
    if (state) return stateTone(state);
    return 'stale';
  });
  const title = $derived(sib ? (sib.mechanismPath.split('/').pop() ?? sib.mechanismPath) : '');
</script>

{#if !anchor}
  <div class="problem">Bind any signal from a mechanism</div>
{:else if !sib}
  <div class="problem">{anchor.path} is not part of a mechanism</div>
{:else}
  <div class="card">
    <header>
      <span class="name" title={sib.mechanismPath}>{title}</span>
      <span class="ui-badge" data-tone={tone}>{state || 'no state'}</span>
    </header>

    <div class="reason" data-tone={tone}>
      <span class="dot ui-dot" data-tone={tone}></span>
      <span class="reason-text" title={reason}>{reason || (state ? '' : 'No State or Reason signal')}</span>
    </div>

    <dl>
      <dt>Stage</dt>
      <dd class="ui-mono">{stage || '--'}</dd>
      <dt>Action</dt>
      <dd class="ui-mono">
        {#if activeActions.length > 0}
          {activeActions.map((s) => s.name).join(', ')}
        {:else}
          idle
        {/if}
      </dd>
      <dt>Faults</dt>
      <dd>
        {#if faulted.length > 0}
          {#each faulted as f (f.signalId)}<span class="ui-badge" data-tone="fault">{f.name}</span>{/each}
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
    justify-content: space-between;
    gap: 8px;
    min-width: 0;
  }
  .name {
    font-size: var(--fs-md);
    font-weight: 600;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .reason {
    display: flex;
    align-items: center;
    gap: 8px;
    min-height: 28px;
    padding: 6px 10px;
    border-radius: var(--radius);
    background: var(--surface-3);
    color: var(--text);
    font-size: var(--fs-sm);
  }
  .reason[data-tone='ok'] {
    background: var(--ok-soft);
    color: var(--ok);
  }
  .reason[data-tone='warn'] {
    background: var(--warn-soft);
    color: var(--warn);
  }
  .reason[data-tone='fault'] {
    background: var(--fault-soft);
    color: var(--fault);
  }
  .reason[data-tone='info'] {
    background: var(--info-soft);
    color: var(--info);
  }
  .reason-text {
    min-width: 0;
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
