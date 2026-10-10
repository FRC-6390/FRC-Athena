<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import type { SignalDescriptor } from '../../lib/data/types';

  type Props = {
    actions: SignalDescriptor[];
    session: Session;
    live: { readonly current: number };
  };

  let { actions, session, live }: Props = $props();

  function replyOf(signal: SignalDescriptor) {
    void live.current;
    return session.store.reply(signal.signalId);
  }

  async function run(signal: SignalDescriptor) {
    await session.commands.run(signal.signalId);
  }

  async function cancel(signal: SignalDescriptor) {
    await session.commands.cancel(signal.signalId);
  }
</script>

<section class="card" aria-labelledby="actions-heading">
  <h3 id="actions-heading">Actions</h3>
  {#if actions.length === 0}
    <p class="empty">No actions published.</p>
  {:else}
    <ul class="actions">
      {#each actions as action (action.signalId)}
        <li>
          <span class="name" title={action.path}>{action.name}</span>
          <button type="button" onclick={() => run(action)}>Run</button>
          <button type="button" class="secondary" onclick={() => cancel(action)}>Cancel</button>
          {#if replyOf(action) && (replyOf(action)?.op === 'run' || replyOf(action)?.op === 'cancel')}
            {@const reply = replyOf(action)}
            <span class="feedback" class:rejected={!reply?.accepted}>
              {reply?.op === 'run' ? 'run' : 'cancel'}: {reply?.accepted ? 'accepted' : (reply?.reason ?? 'rejected')}
            </span>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}
</section>

<style>
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 12px;
    display: grid;
    gap: 8px;
  }
  h3 {
    margin: 0;
    font-size: 13px;
    font-family: var(--font-display);
    color: var(--text-strong);
  }
  .actions {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 6px;
  }
  .actions li {
    display: flex;
    flex-wrap: wrap;
    gap: 6px;
    align-items: center;
  }
  .name {
    flex: 1;
    min-width: 120px;
    font-family: var(--font-mono);
    font-size: 12px;
  }
  button {
    background: var(--brand-strong);
    color: var(--text-strong);
    border: 1px solid var(--brand);
    border-radius: 6px;
    padding: 4px 10px;
    font: inherit;
    cursor: pointer;
  }
  button.secondary {
    background: var(--surface-3);
    border-color: var(--border-subtle);
  }
  .feedback {
    font-size: 11px;
    color: #86efac;
    width: 100%;
  }
  .feedback.rejected {
    color: var(--danger);
  }
  .empty {
    margin: 0;
    color: var(--text-soft);
    font-size: 12px;
  }
</style>
