<script lang="ts">
  import { gainLabel } from '../../../lib/mechanisms/model';
  import type { WidgetContext } from '../../../lib/board/types';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { formatNumber } from '../../../lib/format';
  import { confirmReason, describeChange, parseNumberInput } from '../../../lib/board/widgets/controls-numeric';
  import { findRestoreAction } from '../../../lib/board/widgets/controls-pid';
  import Feedback from './parts/Feedback.svelte';
  import ConfirmBar from './parts/ConfirmBar.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const gains = $derived(ctx.bindings.gains ?? []);
  const restore = $derived.by(() => {
    void ctx.live.current;
    return ctx.bindings.restore?.[0] ?? findRestoreAction(gains, ctx.session.store.allSignals());
  });
  const restoreReply = $derived.by(() => {
    void ctx.live.current;
    const r = restore ? ctx.reply(restore) : undefined;
    return r && (r.op === 'run' || r.op === 'cancel') ? r : undefined;
  });
  const now = $derived.by(() => {
    void ctx.live.current;
    return Date.now();
  });

  function robotValue(s: SignalDescriptor): number | null {
    void ctx.live.current;
    const v = ctx.latest(s);
    return typeof v === 'number' ? v : typeof v === 'bigint' ? Number(v) : null;
  }
  function writeReply(s: SignalDescriptor) {
    void ctx.live.current;
    const r = ctx.reply(s);
    return r && r.op === 'write' ? r : undefined;
  }

  // One row is edited at a time. `pendingPath` marks the row whose write is in flight.
  let editPath = $state<string | null>(null);
  let draft = $state('');
  let error = $state('');
  let pendingPath = $state<string | null>(null);
  let confirm = $state<{ path: string; value: number; reason: string } | null>(null);
  let restoreArmed = $state(false);

  /** Focuses the edit field when a row opens, in place of `autofocus` (which a11y lint rejects). */
  function focusNode(node: HTMLElement) {
    node.focus();
  }

  function startEdit(s: SignalDescriptor) {
    if (ctx.editing) return;
    editPath = s.path;
    draft = robotValue(s) === null ? '' : String(robotValue(s));
    error = '';
    confirm = null;
  }

  function cancelEdit() {
    editPath = null;
    error = '';
    confirm = null;
  }

  async function send(s: SignalDescriptor, value: number) {
    pendingPath = s.path;
    try {
      await ctx.write(s, value);
    } finally {
      pendingPath = null;
      if (editPath === s.path) editPath = null;
      confirm = null;
    }
  }

  function commit(s: SignalDescriptor) {
    const value = parseNumberInput(draft);
    if (value === null) {
      error = 'Enter a number';
      return;
    }
    const reason = confirmReason(robotValue(s), value);
    if (reason) confirm = { path: s.path, value, reason };
    else void send(s, value);
  }

  function confirmText(s: SignalDescriptor, value: number, reason: string): string {
    if (reason === 'unknown') return `Send ${value} to ${s.name}? The robot value is unknown.`;
    return `Send ${s.name} ${describeChange(robotValue(s), value)}?`;
  }

  async function runRestore() {
    if (!restore || ctx.editing) return;
    restoreArmed = false;
    await ctx.run(restore);
  }
</script>

<div class="pid">
  <div class="table" role="table" aria-label="PID gains">
    <div class="head" role="row">
      <span role="columnheader">Gain</span>
      <span role="columnheader" class="num">Robot</span>
      <span role="columnheader" class="num">Set</span>
    </div>
    {#each gains as gain (gain.path)}
      {@const reply = writeReply(gain)}
      {@const locked = ctx.locked(gain)}
      <div class="row" role="row" title={gain.path}>
        <span class="name" role="cell">{gainLabel(gain)}</span>
        <span class="num ui-mono robot" role="cell">{robotValue(gain) === null ? '--' : formatNumber(robotValue(gain) ?? 0)}</span>
        <span class="cell-edit" role="cell">
          {#if editPath === gain.path}
            <input
              class="ui-input value-input ui-mono"
              aria-label={`New value for ${gain.name}`}
              value={draft}
              use:focusNode
              disabled={ctx.editing}
              oninput={(e) => {
                draft = e.currentTarget.value;
                confirm = null;
              }}
              onkeydown={(e) => {
                if (e.key === 'Enter') commit(gain);
                if (e.key === 'Escape') cancelEdit();
              }}
            />
          {:else}
            <button
              type="button"
              class="ui-btn value-btn ui-mono"
              disabled={ctx.editing || !gain.writable}
              onclick={() => startEdit(gain)}
              title="Edit {gain.name}"
            >{pendingPath === gain.path ? 'sending' : 'edit'}</button>
          {/if}
        </span>
        <span class="meta">
          <Feedback {reply} {locked} pending={pendingPath === gain.path} {now} />
        </span>
        {#if editPath === gain.path}
          <span class="edit-actions">
            <button type="button" class="ui-btn" data-variant="primary" data-size="sm" disabled={ctx.editing} onclick={() => commit(gain)}>Send</button>
            <button type="button" class="ui-btn" data-variant="ghost" data-size="sm" onclick={cancelEdit}>Cancel</button>
          </span>
          {#if error}<span class="error">{error}</span>{/if}
        {/if}
        {#if confirm?.path === gain.path}
          <div class="confirm">
            <ConfirmBar text={confirmText(gain, confirm.value, confirm.reason)} onconfirm={() => confirm && send(gain, confirm.value)} oncancel={() => (confirm = null)} />
          </div>
        {/if}
      </div>
    {/each}
    {#if gains.length === 0}
      <div class="empty">Bind writable gains (kP, kI, kD, ...)</div>
    {/if}
  </div>

  {#if restore}
    <div class="footer">
      {#if restoreArmed}
        <span class="ask">Restore {restore.name} on the robot?</span>
        <button type="button" class="ui-btn" data-variant="primary" data-size="sm" disabled={ctx.editing} onclick={runRestore}>Restore</button>
        <button type="button" class="ui-btn" data-size="sm" onclick={() => (restoreArmed = false)}>Cancel</button>
      {:else}
        <button type="button" class="ui-btn" data-size="sm" disabled={ctx.editing} onclick={() => (restoreArmed = true)}>Restore defaults</button>
        {#if restoreReply && !restoreReply.accepted}
          <span class="rejected" title={restoreReply.reason}>restore rejected{restoreReply.reason ? `: ${restoreReply.reason}` : ''}</span>
        {/if}
      {/if}
    </div>
  {/if}
</div>

<style>
  .pid {
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    height: 100%;
    min-width: 0;
  }
  .table {
    overflow: auto;
    min-height: 0;
    padding: 0 6px;
  }
  .head,
  .row {
    display: grid;
    grid-template-columns: minmax(60px, 1fr) minmax(60px, 0.9fr) minmax(64px, auto);
    align-items: center;
    gap: 4px 8px;
    min-width: 0;
  }
  .head {
    position: sticky;
    top: 0;
    z-index: 1;
    padding: 6px 0;
    background: var(--surface);
    font-size: var(--fs-xs);
    color: var(--text-faint);
    text-transform: uppercase;
    letter-spacing: 0.04em;
    border-bottom: 1px solid var(--border);
  }
  .row {
    padding: 5px 0;
    border-bottom: 1px solid var(--border);
    font-size: var(--fs-sm);
  }
  .name {
    font-family: var(--font-mono);
    font-weight: 600;
    color: var(--text-strong);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .num {
    text-align: right;
  }
  .robot {
    color: var(--text);
  }
  .cell-edit {
    display: flex;
    justify-content: flex-end;
    min-width: 0;
  }
  .value-btn {
    height: 24px;
    min-width: 60px;
    padding: 0 8px;
    font-size: var(--fs-xs);
  }
  .value-input {
    width: 100%;
    min-width: 0;
    height: 26px;
    text-align: right;
  }
  .meta {
    grid-column: 1 / 2;
    min-width: 0;
  }
  .edit-actions {
    display: inline-flex;
    gap: 4px;
    justify-self: end;
  }
  .confirm {
    grid-column: 1 / -1;
  }
  .error {
    grid-column: 1 / -1;
    font-size: var(--fs-xs);
    color: var(--fault);
  }
  .empty {
    display: grid;
    place-items: center;
    height: 100%;
    min-height: 60px;
    color: var(--text-faint);
    font-size: var(--fs-sm);
  }
  .footer {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 6px;
    padding: 6px 8px;
    border-top: 1px solid var(--border);
    font-size: var(--fs-xs);
  }
  .ask {
    flex: 1;
    min-width: 0;
    color: var(--text);
  }
  .rejected {
    color: var(--fault);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    max-width: 100%;
  }
</style>
