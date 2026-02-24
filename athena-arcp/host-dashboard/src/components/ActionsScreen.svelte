<script lang="ts">
  import type { SignalRow } from '../lib/arcp';

  type Props = {
    actions: SignalRow[];
    onTriggerAction: (signalId: number) => void;
    onSelectSignal: (signalId: number) => void;
  };

  let { actions, onTriggerAction, onSelectSignal }: Props = $props();
</script>

<section class="actions-screen panel">
  <header>
    <h2>Action Center</h2>
    <p>Trigger command-style signals exposed by the ARCP runtime.</p>
  </header>

  {#if actions.length === 0}
    <div class="empty">
      <p>No action signals found in the current manifest.</p>
    </div>
  {:else}
    <div class="action-list">
      {#each actions as signal}
        <article class="action-card">
          <div class="copy">
            <h3>{signal.path}</h3>
            <p>id #{signal.signal_id} · {signal.signal_type}</p>
          </div>
          <div class="buttons">
            <button
              class="btn"
              onclick={() => onSelectSignal(signal.signal_id)}
            >
              Inspect
            </button>
            <button
              class="btn btn-danger"
              onclick={() => onTriggerAction(signal.signal_id)}
            >
              Trigger
            </button>
          </div>
        </article>
      {/each}
    </div>
  {/if}
</section>

<style>
  .actions-screen {
    padding: 0.74rem;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.7rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 0.98rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.18rem 0 0;
    color: var(--text-soft);
    font-size: 0.76rem;
  }

  .action-list {
    display: grid;
    gap: 0.48rem;
    min-height: 0;
    overflow: auto;
    padding-right: 0.22rem;
  }

  .action-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.54rem 0.6rem;
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.6rem;
  }

  .copy {
    min-width: 0;
  }

  .copy h3 {
    margin: 0;
    font-size: 0.8rem;
    color: var(--text-strong);
    line-height: 1.2;
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .copy p {
    margin: 0.12rem 0 0;
    font-size: 0.72rem;
    color: var(--text-soft);
  }

  .buttons {
    display: flex;
    gap: 0.4rem;
  }

  .empty {
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.65rem;
    color: var(--text-soft);
    font-size: 0.78rem;
  }

  .empty p {
    margin: 0;
  }

  @media (max-width: 820px) {
    .action-card {
      flex-direction: column;
      align-items: stretch;
    }

    .buttons {
      justify-content: flex-end;
    }
  }
</style>
