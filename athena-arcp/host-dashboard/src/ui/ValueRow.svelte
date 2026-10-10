<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faLock, faFloppyDisk, faCheck, faXmark } from '@fortawesome/free-solid-svg-icons';
  import type { Session } from '../lib/data/session';
  import type { SignalDescriptor } from '../lib/data/types';
  import { formatSignalValue, parseInput } from '../lib/format';

  type Props = {
    signal: SignalDescriptor;
    session: Session;
    live: { readonly current: number };
    label?: string;
    /** Optional tone for the value (e.g. warn for a hot motor). */
    tone?: 'ok' | 'warn' | 'fault' | null;
  };

  let { signal, session, live, label, tone = null }: Props = $props();

  let editing = $state(false);
  let draft = $state('');
  let error = $state<string | null>(null);
  let input = $state<HTMLInputElement | null>(null);

  const latest = $derived.by(() => {
    void live.current;
    return session.store.latestSample(signal.signalId)?.value;
  });
  const reply = $derived.by(() => {
    void live.current;
    const r = session.store.reply(signal.signalId);
    return r && r.op === 'write' ? r : undefined;
  });
  const locked = $derived.by(() => {
    void live.current;
    return session.store.lockStateOf(signal.signalId) === 'locked';
  });
  const isBool = $derived(signal.type === 'bool');

  function begin() {
    if (!signal.writable) return;
    if (isBool) {
      void commit(String(!(latest === true)));
      return;
    }
    draft = latest === undefined ? '' : String(latest);
    error = null;
    editing = true;
    queueMicrotask(() => input?.select());
  }

  /** Always sent; the robot decides (E5). A rejection shows the robot's reason and its actual value. */
  async function commit(raw: string) {
    const value = parseInput(signal.type, raw);
    if (value === null) {
      error = `not a valid ${signal.type}`;
      return;
    }
    editing = false;
    error = null;
    await session.commands.write(signal.signalId, value);
  }

  function onkey(event: KeyboardEvent) {
    if (event.key === 'Enter') void commit(draft);
    if (event.key === 'Escape') editing = false;
  }
</script>

<div class="row" data-path={signal.path}>
  <span class="key" title={signal.path}>{label ?? signal.name}</span>
  <span class="meta">
    {#if locked}<span class="lock" title={`Locked (${signal.lockGroup.split('.').pop()})`}><FontAwesomeIcon icon={faLock} /></span>{/if}
    {#if signal.persisted}<span class="saved" title="Persisted on the robot"><FontAwesomeIcon icon={faFloppyDisk} /></span>{/if}
  </span>
  {#if editing}
    <input
      class="ui-input edit"
      bind:this={input}
      bind:value={draft}
      onkeydown={onkey}
      onblur={() => (editing = false)}
      aria-label="new value for {signal.name}"
    />
  {:else}
    <button
      type="button"
      class="value ui-mono"
      class:writable={signal.writable}
      data-tone={tone}
      disabled={!signal.writable}
      onclick={begin}
      title={signal.writable ? (isBool ? 'Click to toggle' : 'Click to edit') : undefined}
    >
      {formatSignalValue(signal.type, latest)}{#if signal.unit}<span class="unit">{signal.unit}</span>{/if}
    </button>
  {/if}
  {#if error}
    <span class="reply bad" title={error}><FontAwesomeIcon icon={faXmark} /></span>
  {:else if reply}
    <span class="reply" class:bad={!reply.accepted} title={reply.accepted ? 'Accepted by robot' : reply.reason}>
      <FontAwesomeIcon icon={reply.accepted ? faCheck : faXmark} />
    </span>
  {:else}
    <span class="reply"></span>
  {/if}
</div>
{#if reply && !reply.accepted}
  <div class="rejection">{reply.reason || 'Rejected by robot'}</div>
{/if}

<style>
  .row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto auto 16px;
    align-items: center;
    gap: var(--sp-2);
    min-height: 30px;
    padding: 0 var(--sp-3);
    border-top: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  .key {
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .meta {
    display: inline-flex;
    gap: 6px;
    font-size: 10px;
  }
  .lock {
    color: var(--locked);
  }
  .saved {
    color: var(--text-faint);
  }
  .value {
    justify-self: end;
    border: 1px solid transparent;
    border-radius: var(--radius-sm);
    background: transparent;
    padding: 2px 6px;
    color: var(--text-strong);
    font-size: var(--fs-sm);
    text-align: right;
    cursor: default;
  }
  .value.writable {
    cursor: pointer;
  }
  .value.writable:hover {
    border-color: var(--border-subtle);
    background: var(--bg-1);
  }
  .value[data-tone='warn'] {
    color: var(--warn);
  }
  .value[data-tone='fault'] {
    color: var(--fault);
  }
  .value:disabled {
    opacity: 1;
  }
  .unit {
    margin-left: 4px;
    color: var(--text-faint);
  }
  .edit {
    width: 110px;
    justify-self: end;
    text-align: right;
  }
  .reply {
    font-size: 11px;
    color: var(--ok);
    text-align: center;
  }
  .reply.bad {
    color: var(--fault);
  }
  .rejection {
    padding: 0 var(--sp-3) 6px;
    font-size: var(--fs-xs);
    color: var(--fault);
  }
</style>
