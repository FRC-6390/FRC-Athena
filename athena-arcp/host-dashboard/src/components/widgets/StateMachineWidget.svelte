<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readStateMachineConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
    onSendAction: (signalId: number) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet, onSendAction }: Props = $props();

  let goalDraft = $state('');

  let goalSeedSignalId = $state<number | null>(null);

  const config = $derived(readStateMachineConfig(configRaw, signal, signals));

  function rowFor(signalId: number | null): SignalRow | null {
    if (signalId === null) return null;
    return signalById.get(signalId) ?? null;
  }

  function textFor(row: SignalRow | null): string | null {
    if (!row) return null;
    const value = row.value.trim();
    return value.length > 0 ? value : null;
  }

  function boolFor(row: SignalRow | null): boolean | null {
    if (!row) return null;
    const normalized = row.value.trim().toLowerCase();
    if (normalized === 'true' || normalized === '1' || normalized === 'yes' || normalized === 'on') {
      return true;
    }
    if (normalized === 'false' || normalized === '0' || normalized === 'no' || normalized === 'off') {
      return false;
    }
    return null;
  }

  function numberFor(row: SignalRow | null): number | null {
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function parseStringArray(raw: string | null): string[] {
    const value = raw?.trim() ?? '';
    if (!value || value === '-') return [];

    try {
      const parsed = JSON.parse(value) as unknown;
      const candidates = Array.isArray(parsed)
        ? parsed
        : parsed && typeof parsed === 'object' && 'values' in parsed && Array.isArray((parsed as { values: unknown[] }).values)
          ? (parsed as { values: unknown[] }).values
          : null;
      if (candidates) {
        return candidates
          .map((entry) => String(entry).trim())
          .filter((entry) => entry.length > 0)
          .slice(0, 64);
      }
    } catch {
      // fall through
    }

    if (value.includes('|')) {
      return value
        .split('|')
        .map((entry) => entry.trim())
        .filter((entry) => entry.length > 0)
        .slice(0, 64);
    }

    if (value.includes(',')) {
      return value
        .split(',')
        .map((entry) => entry.trim())
        .filter((entry) => entry.length > 0)
        .slice(0, 64);
    }

    if (value.startsWith('[') && value.endsWith(']')) {
      return value
        .slice(1, -1)
        .split(/\s+/)
        .map((entry) => entry.replace(/["']/g, '').trim())
        .filter((entry) => entry.length > 0)
        .slice(0, 64);
    }

    return [value];
  }

  function normalizeState(value: string): string {
    return value.trim().toLowerCase();
  }

  function parentPath(path: string): string {
    const slash = path.lastIndexOf('/');
    if (slash <= 0) return '';
    return path.slice(0, slash);
  }

  function leafPath(path: string): string {
    const slash = path.lastIndexOf('/');
    if (slash < 0) return path;
    return path.slice(slash + 1);
  }

  function normalizeToken(value: string): string {
    return value.toLowerCase().replace(/[^a-z0-9]+/g, '');
  }

  function isWritable(row: SignalRow | null): row is SignalRow {
    return !!row && row.access === 'write';
  }

  function isAction(row: SignalRow | null): row is SignalRow {
    return !!row && (row.access === 'invoke' || row.kind === 'command');
  }

  function formatBool(value: boolean | null): string {
    if (value === null) return '--';
    return value ? 'true' : 'false';
  }

  function formatNumber(value: number | null): string {
    if (value === null) return '--';
    return Number.isInteger(value) ? String(value) : value.toFixed(2);
  }

  function applyGoal(queueMode: boolean) {
    if (!goalStateRow || !isWritable(goalStateRow)) return;
    const next = goalDraft.trim();
    if (!next) return;

    if (implicitAppendModeRow && isWritable(implicitAppendModeRow)) {
      onSendSet(implicitAppendModeRow.signal_id, queueMode ? 'true' : 'false');
    }

    onSendSet(goalStateRow.signal_id, next);
  }

  function queueGoal() {
    applyGoal(true);
  }

  function forceGoal() {
    applyGoal(false);
  }

  function clearQueue() {
    if (!clearQueueRow || !isAction(clearQueueRow)) return;
    onSendAction(clearQueueRow.signal_id);
  }

  const currentStateRow = $derived(rowFor(config.currentStateSignalId));
  const goalStateRow = $derived(rowFor(config.goalStateSignalId));
  const nextStateRow = $derived(rowFor(config.nextStateSignalId));
  const queueRow = $derived(rowFor(config.queueSignalId));
  const atGoalRow = $derived(rowFor(config.atGoalSignalId));
  const transitionCountRow = $derived(rowFor(config.transitionCountSignalId));
  const lastTransitionRow = $derived(rowFor(config.lastTransitionSignalId));
  const availableStatesRow = $derived(rowFor(config.availableStatesSignalId));
  const clearQueueRow = $derived(rowFor(config.clearQueueCommandSignalId));

  const implicitAppendModeRow = $derived.by(() => {
    const anchorParent = parentPath(goalStateRow?.path ?? signal.path);
    const scoped = signals.filter((entry) => parentPath(entry.path) === anchorParent);
    const pool = scoped.length > 0 ? scoped : signals;

    for (const entry of pool) {
      if (entry.signal_type !== 'bool' || entry.access !== 'write') continue;
      const token = normalizeToken(leafPath(entry.path));
      if (token === 'appendmode' || token === 'append' || token === 'queuemode') {
        return entry;
      }
    }

    for (const entry of pool) {
      if (entry.signal_type !== 'bool' || entry.access !== 'write') continue;
      const token = normalizeToken(leafPath(entry.path));
      if (token.includes('append') || token.includes('queuemode')) {
        return entry;
      }
    }

    return null;
  });

  const currentState = $derived(textFor(currentStateRow) ?? '--');
  const goalState = $derived(textFor(goalStateRow) ?? '--');
  const nextState = $derived(textFor(nextStateRow) ?? '--');
  const queueItems = $derived(parseStringArray(textFor(queueRow)));
  const atGoal = $derived(boolFor(atGoalRow));
  const transitionCount = $derived(numberFor(transitionCountRow));
  const lastTransition = $derived(textFor(lastTransitionRow) ?? '--');

  const availableStates = $derived.by(() => {
    const explicit = parseStringArray(textFor(availableStatesRow));
    if (explicit.length > 0) return explicit;

    const fallback = [currentState, goalState, nextState]
      .map((entry) => entry.trim())
      .filter((entry) => entry.length > 0 && entry !== '--');
    return Array.from(new Set(fallback));
  });

  const goalMatched = $derived.by(() => {
    if (atGoal !== null) return atGoal;
    const current = normalizeState(currentState);
    const goal = normalizeState(goalState);
    if (!current || !goal || current === '--' || goal === '--') return false;
    return current === goal;
  });

  const writableControlsCount = $derived(
    Number(isWritable(goalStateRow)) +
      Number(isAction(clearQueueRow))
  );

  $effect(() => {
    if (!goalStateRow || !isWritable(goalStateRow)) {
      goalSeedSignalId = null;
      return;
    }
    if (goalSeedSignalId === goalStateRow.signal_id) return;
    goalSeedSignalId = goalStateRow.signal_id;

    const direct = textFor(goalStateRow);
    if (direct) {
      goalDraft = direct;
      return;
    }

    goalDraft = availableStates[0] ?? '';
  });

</script>

<div class="state-machine-root">
  <section class="summary-card">
    <div class="chip-row">
      <span class={`chip ${atGoal === true ? 'ok' : atGoal === false ? 'warn' : ''}`}>
        at_goal: {formatBool(atGoal)}
      </span>
      <span class="chip">transitions: {formatNumber(transitionCount)}</span>
    </div>

    <div class="state-grid">
      <article>
        <span>Current</span>
        <strong>{currentState}</strong>
      </article>
      <article class:goal-match={goalMatched} class:goal-miss={!goalMatched}>
        <span>Goal</span>
        <strong>{goalState}</strong>
      </article>
      <article>
        <span>Next</span>
        <strong>{nextState}</strong>
      </article>
      <article>
        <span>Last transition</span>
        <strong class="wrap">{lastTransition}</strong>
      </article>
    </div>

    <div class="queue-block">
      <h5>Queue</h5>
      {#if queueItems.length > 0}
        <ul>
          {#each queueItems as item, index (`queue-${index}-${item}`)}
            <li>{item}</li>
          {/each}
        </ul>
      {:else}
        <p>Queue is empty.</p>
      {/if}
    </div>
  </section>

  <section class="controls-card">
    <h5>Controls</h5>

    {#if isWritable(goalStateRow)}
      <div class="control-row goal-row">
        <span>Goal state</span>
        <select
          value={goalDraft}
          onchange={(event) => {
            goalDraft = (event.currentTarget as HTMLSelectElement).value;
          }}
        >
          {#if availableStates.length === 0}
            <option value="">No options</option>
          {:else}
            {#each availableStates as stateOption (`goal-${stateOption}`)}
              <option value={stateOption}>{stateOption}</option>
            {/each}
          {/if}
        </select>
        <div class="goal-actions">
          <button class="btn btn-primary" onclick={queueGoal}>Queue</button>
          <button class="btn btn-danger" onclick={forceGoal}>Force</button>
        </div>
      </div>
    {/if}

    {#if isAction(clearQueueRow)}
      <div class="control-row clear-row">
        <span>Queue</span>
        <div class="hint-inline">Remove all pending states</div>
        <button class="btn btn-danger" onclick={clearQueue}>Clear Queue</button>
      </div>
    {/if}

    {#if writableControlsCount === 0}
      <p class="hint">No writable state-machine controls are mapped.</p>
    {/if}
  </section>
</div>

<style>
  .state-machine-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: minmax(0, 1fr) auto;
    gap: 0.34rem;
  }

  .summary-card,
  .controls-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: linear-gradient(180deg, rgba(42, 50, 66, 0.58), rgba(23, 29, 41, 0.74));
    padding: 0.4rem;
    min-height: 0;
  }

  .summary-card {
    display: grid;
    gap: 0.34rem;
    grid-template-rows: auto auto minmax(0, 1fr);
  }

  .chip-row {
    display: flex;
    flex-wrap: wrap;
    gap: 0.24rem;
  }

  .chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.12rem 0.36rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    background: var(--surface-3);
    text-transform: uppercase;
    letter-spacing: 0.02em;
  }

  .chip.ok {
    border-color: rgba(52, 211, 153, 0.46);
    color: #bbf7d0;
    background: rgba(20, 83, 45, 0.5);
  }

  .chip.warn {
    border-color: rgba(248, 113, 113, 0.48);
    color: #fecaca;
    background: rgba(127, 29, 29, 0.42);
  }

  .state-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.3rem;
  }

  .state-grid article {
    border: 1px solid rgba(153, 164, 180, 0.3);
    border-radius: 7px;
    background: rgba(20, 27, 39, 0.7);
    padding: 0.26rem 0.34rem;
    display: grid;
    gap: 0.1rem;
  }

  .state-grid article.goal-match {
    border-color: rgba(52, 211, 153, 0.58);
    background: rgba(20, 83, 45, 0.45);
  }

  .state-grid article.goal-miss {
    border-color: rgba(248, 113, 113, 0.54);
    background: rgba(127, 29, 29, 0.36);
  }

  .state-grid span {
    font-size: 0.64rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  .state-grid strong {
    font-size: 0.84rem;
    color: var(--text-strong);
    font-family: var(--font-mono);
    line-height: 1.2;
    white-space: nowrap;
    text-overflow: ellipsis;
    overflow: hidden;
  }

  .state-grid strong.wrap {
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .queue-block {
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    gap: 0.16rem;
  }

  .queue-block h5,
  .controls-card h5 {
    margin: 0;
    font-size: 0.72rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }

  .queue-block ul {
    margin: 0;
    padding: 0 0 0 1rem;
    display: grid;
    gap: 0.14rem;
    overflow: auto;
  }

  .queue-block li {
    font-size: 0.73rem;
    color: var(--text-strong);
    font-family: var(--font-mono);
    overflow-wrap: anywhere;
  }

  .queue-block p,
  .hint,
  .hint-inline {
    margin: 0;
    font-size: 0.7rem;
    color: var(--text-soft);
  }

  .controls-card {
    display: grid;
    gap: 0.26rem;
  }

  .control-row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(0, 1.3fr) auto;
    align-items: center;
    gap: 0.3rem;
  }

  .control-row > span {
    font-size: 0.68rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  .goal-row .goal-actions {
    display: inline-flex;
    gap: 0.24rem;
  }

  .control-row select {
    width: 100%;
    min-width: 0;
    font-size: 0.7rem;
    padding: 0.2rem 0.28rem;
    background: #1f2838;
    color: var(--text-strong);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
  }

  .clear-row .hint-inline {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  @media (max-width: 880px) {
    .control-row {
      grid-template-columns: 1fr;
    }

    .goal-row .goal-actions {
      justify-content: flex-start;
    }
  }
</style>
