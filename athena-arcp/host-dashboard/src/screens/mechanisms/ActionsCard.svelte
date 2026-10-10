<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faPlay, faStop } from '@fortawesome/free-solid-svg-icons';
  import type { Session } from '../../lib/data/session';
  import type { SignalDescriptor } from '../../lib/data/types';

  type Props = {
    actions: SignalDescriptor[];
    session: Session;
    live: { readonly current: number };
  };

  let { actions, session, live }: Props = $props();

  function running(signal: SignalDescriptor): boolean {
    void live.current;
    return session.store.latestSample(signal.signalId)?.value === true;
  }

  function reply(signal: SignalDescriptor) {
    void live.current;
    const r = session.store.reply(signal.signalId);
    return r && (r.op === 'run' || r.op === 'cancel') ? r : undefined;
  }

  const label = (name: string) => {
    const words = name.replace(/([a-z])([A-Z])/g, '$1 $2').toLowerCase();
    return words.charAt(0).toUpperCase() + words.slice(1);
  };
</script>

<section class="ui-panel">
  <header class="ui-panel-head">
    <h3 class="ui-panel-title">Actions</h3>
    <span class="ui-spacer"></span>
    <span class="count ui-mono">{actions.length}</span>
  </header>
  {#if actions.length === 0}
    <p class="empty">No actions published.</p>
  {:else}
    <ul>
      {#each actions as action (action.signalId)}
        {@const isRunning = running(action)}
        {@const r = reply(action)}
        <li class:running={isRunning}>
          <span class="state" class:on={isRunning}></span>
          <span class="name" title={action.path}>{label(action.name)}</span>
          {#if r && !r.accepted}
            <span class="ui-badge" data-tone="fault" title={r.reason}>rejected</span>
          {/if}
          {#if isRunning}
            <button type="button" class="ui-btn" data-size="sm" onclick={() => session.commands.cancel(action.signalId)}>
              <FontAwesomeIcon icon={faStop} /> Cancel
            </button>
          {:else}
            <button type="button" class="ui-btn" data-size="sm" onclick={() => session.commands.run(action.signalId)}>
              <FontAwesomeIcon icon={faPlay} /> Run
            </button>
          {/if}
        </li>
      {/each}
    </ul>
  {/if}
</section>

<style>
  ul {
    list-style: none;
    margin: 0;
    padding: 0;
  }
  li {
    display: grid;
    grid-template-columns: 8px minmax(0, 1fr) auto auto;
    align-items: center;
    gap: var(--sp-2);
    min-height: 36px;
    padding: 0 var(--sp-3);
    border-top: 1px solid var(--border);
  }
  li:first-child {
    border-top: none;
  }
  li.running {
    background: var(--ok-soft);
  }
  .state {
    width: 6px;
    height: 6px;
    border-radius: 50%;
    background: var(--border-emphasis);
  }
  .state.on {
    background: var(--ok);
  }
  .name {
    font-size: var(--fs-sm);
    color: var(--text);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  li .ui-badge {
    grid-column: 3;
  }
  li .ui-btn {
    grid-column: 4;
  }
  .count {
    font-size: var(--fs-xs);
    color: var(--text-faint);
  }
  .empty {
    margin: 0;
    padding: var(--sp-3);
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
</style>
