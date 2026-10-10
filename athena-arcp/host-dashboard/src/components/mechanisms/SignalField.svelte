<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { SignalDescriptor } from '../../lib/data/types';
  import { formatValue, parseInput } from '../../lib/format';

  type Props = {
    signal: SignalDescriptor;
    session: Session;
    live: { readonly current: number };
    label?: string;
  };

  let { signal, session, live, label }: Props = $props();

  let draft = $state('');
  let localError = $state<string | null>(null);
  let sending = $state(false);

  const latest = $derived.by(() => {
    void live.current;
    return session.store.latestSample(signal.signalId)?.value;
  });
  const reply = $derived.by(() => {
    void live.current;
    return session.store.reply(signal.signalId);
  });
  const editable = $derived(signal.writable);
  const isBool = $derived(signal.type === 'bool');

  /** Writes are always sent. Lock state is shown, never used to block the send (E5). */
  async function send(event: SubmitEvent) {
    event.preventDefault();
    const value = parseInput(signal.type, isBool && draft === '' ? 'true' : draft);
    if (value === null) {
      localError = `Enter a valid ${signal.type} value`;
      return;
    }
    localError = null;
    sending = true;
    try {
      await session.commands.write(signal.signalId, value);
    } finally {
      sending = false;
    }
  }
</script>

<div class="field" class:field-locked={signal.lockState === 'locked'} data-path={signal.path}>
  <div class="field-head">
    <span class="field-name" title={signal.path}>{label ?? signal.localPath}</span>
    {#if signal.writable}
      <span class="badge lock lock-{signal.lockState}" title={signal.lockGroup ? `lock group ${signal.lockGroup}` : 'no lock group'}>
        {signal.lockState === 'locked' ? 'locked' : signal.lockState === 'open' ? 'open' : 'writable'}
      </span>
    {/if}
    {#if signal.persisted}
      <span class="badge">saved</span>
    {/if}
  </div>
  <div class="field-value">
    <span class="value">{formatValue(latest)}</span>
    {#if signal.unit}<span class="unit">{signal.unit}</span>{/if}
  </div>

  {#if editable}
    <form class="field-edit" onsubmit={send}>
      {#if isBool}
        <select bind:value={draft} aria-label="new value for {signal.name}">
          <option value="true">true</option>
          <option value="false">false</option>
        </select>
      {:else}
        <input
          type={signal.type === 'string' ? 'text' : 'number'}
          step="any"
          bind:value={draft}
          placeholder={formatValue(latest)}
          aria-label="new value for {signal.name}"
        />
      {/if}
      <button type="submit" disabled={sending}>Send</button>
    </form>
  {/if}

  {#if localError}
    <p class="feedback rejected">{localError}</p>
  {:else if reply && reply.op === 'write'}
    <p class="feedback" class:rejected={!reply.accepted}>
      {reply.accepted ? 'accepted by robot' : (reply.reason ?? 'rejected by robot')}
    </p>
  {/if}
</div>

<style>
  .field {
    display: grid;
    gap: 4px;
    padding: 8px 10px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 8px;
    min-width: 0;
  }
  .field-locked {
    border-color: var(--border-subtle);
  }
  .field-head {
    display: flex;
    gap: 6px;
    align-items: center;
    min-width: 0;
  }
  .field-name {
    font-size: 12px;
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    flex: 1;
    min-width: 0;
  }
  .field-value {
    display: flex;
    align-items: baseline;
    gap: 4px;
    font-family: var(--font-mono);
  }
  .value {
    font-size: 16px;
    color: var(--text-strong);
    overflow-wrap: anywhere;
  }
  .unit {
    font-size: 11px;
    color: var(--text-soft);
  }
  .field-edit {
    display: flex;
    gap: 6px;
  }
  .field-edit input,
  .field-edit select {
    flex: 1;
    min-width: 0;
    background: var(--bg-2);
    color: var(--text);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 4px 6px;
    font: inherit;
  }
  button {
    background: var(--surface-3);
    color: var(--text-strong);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 4px 10px;
    font: inherit;
    cursor: pointer;
  }
  button:disabled {
    opacity: 0.6;
    cursor: default;
  }
  .badge {
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    padding: 1px 5px;
    border-radius: 4px;
    border: 1px solid var(--border);
    color: var(--text-soft);
  }
  .lock-locked {
    color: #fca5a5;
    border-color: #7f1d1d;
  }
  .lock-open {
    color: #86efac;
    border-color: #14532d;
  }
  .feedback {
    margin: 0;
    font-size: 11px;
    color: #86efac;
  }
  .feedback.rejected {
    color: var(--danger);
  }
</style>
