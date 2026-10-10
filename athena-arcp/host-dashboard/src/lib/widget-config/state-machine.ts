import { SignalRow } from '../arcp';
import { WidgetConfigRecord, leafPath } from '../dashboard';
import { findSignalByLeaf, isNtPublishToggleSignal, isObject, normalizeToken, parentPath, resolveSignalIdFromKeys, scopedSignals } from './common';

export type StateMachineWidgetConfig = {
  currentStateSignalId: number | null;
  goalStateSignalId: number | null;
  nextStateSignalId: number | null;
  queueSignalId: number | null;
  atGoalSignalId: number | null;
  transitionCountSignalId: number | null;
  lastTransitionSignalId: number | null;
  availableStatesSignalId: number | null;
  clearQueueCommandSignalId: number | null;
};

export function stateMachineRootPath(path: string): string {
  const parts = path.split('/').filter(Boolean);
  const index = parts.findIndex((part) => {
    const token = normalizeToken(part);
    return token === 'statemachine' || token === 'fsm';
  });
  if (index >= 0) {
    const end = Math.min(parts.length, index + 2);
    return parts.slice(0, end).join('/');
  }
  return parentPath(path);
}

export function findActionSignalByLeaf(
  signals: SignalRow[],
  anchorParent: string,
  candidates: string[]
): SignalRow | null {
  const pool = scopedSignals(signals, anchorParent);
  const actionPool = pool.filter(
    (signal) => signal.access === 'invoke' || signal.kind === 'command'
  );

  const normalizedCandidates = new Set(candidates.map((candidate) => normalizeToken(candidate)));

  for (const signal of actionPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    const token = normalizeToken(leafPath(signal.path));
    if (!token) continue;
    if (normalizedCandidates.has(token)) {
      return signal;
    }
  }

  for (const signal of actionPool) {
    if (isNtPublishToggleSignal(signal)) continue;
    const token = normalizeToken(leafPath(signal.path));
    if (!token) continue;
    for (const candidate of normalizedCandidates) {
      if (token.endsWith(candidate) || token.includes(candidate)) {
        return signal;
      }
    }
  }

  return null;
}

export function defaultStateMachineConfig(
  anchor: SignalRow,
  signals: SignalRow[]
): StateMachineWidgetConfig {
  const root = stateMachineRootPath(anchor.path);
  const commandRoot = root ? `${root}/command` : parentPath(anchor.path);

  const currentStateSignal = findSignalByLeaf(
    signals,
    root,
    ['current_state', 'current', 'state'],
    ['string']
  );
  const goalStateSignal = findSignalByLeaf(
    signals,
    root,
    ['goal_state', 'goal', 'target_state', 'target'],
    ['string']
  );
  const nextStateSignal = findSignalByLeaf(
    signals,
    root,
    ['next_state', 'next'],
    ['string']
  );
  const queueSignal = findSignalByLeaf(
    signals,
    root,
    ['queue', 'pending', 'state_queue'],
    ['string[]', 'string']
  );
  const atGoalSignal = findSignalByLeaf(
    signals,
    root,
    ['at_goal', 'goal_reached'],
    ['bool']
  );
  const transitionCountSignal = findSignalByLeaf(
    signals,
    root,
    ['transition_count', 'transitions', 'count'],
    ['i64', 'f64']
  );
  const lastTransitionSignal = findSignalByLeaf(
    signals,
    root,
    ['last_transition', 'transition'],
    ['string']
  );
  const availableStatesSignal = findSignalByLeaf(
    signals,
    root,
    ['available_states', 'states', 'state_list'],
    ['string[]', 'string']
  );

  const clearQueueCommand = findActionSignalByLeaf(
    signals,
    commandRoot,
    ['clear_queue', 'clear']
  );

  const leaf = normalizeToken(leafPath(anchor.path));
  const currentStateSignalId =
    currentStateSignal?.signal_id ??
    (anchor.signal_type === 'string' && leaf.includes('current') ? anchor.signal_id : null);
  const goalStateSignalId =
    goalStateSignal?.signal_id ??
    (anchor.signal_type === 'string' && leaf.includes('goal') ? anchor.signal_id : null);

  return {
    currentStateSignalId,
    goalStateSignalId,
    nextStateSignalId: nextStateSignal?.signal_id ?? null,
    queueSignalId: queueSignal?.signal_id ?? null,
    atGoalSignalId: atGoalSignal?.signal_id ?? null,
    transitionCountSignalId: transitionCountSignal?.signal_id ?? null,
    lastTransitionSignalId: lastTransitionSignal?.signal_id ?? null,
    availableStatesSignalId: availableStatesSignal?.signal_id ?? null,
    clearQueueCommandSignalId: clearQueueCommand?.signal_id ?? null
  };
}

export function readStateMachineConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): StateMachineWidgetConfig {
  const defaults = defaultStateMachineConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};

  return {
    currentStateSignalId: resolveSignalIdFromKeys(
      base,
      ['currentStateSignalId', 'currentSignalId'],
      defaults.currentStateSignalId,
      signals
    ),
    goalStateSignalId: resolveSignalIdFromKeys(
      base,
      ['goalStateSignalId', 'goalSignalId'],
      defaults.goalStateSignalId,
      signals
    ),
    nextStateSignalId: resolveSignalIdFromKeys(
      base,
      ['nextStateSignalId', 'nextSignalId'],
      defaults.nextStateSignalId,
      signals
    ),
    queueSignalId: resolveSignalIdFromKeys(base, ['queueSignalId'], defaults.queueSignalId, signals),
    atGoalSignalId: resolveSignalIdFromKeys(base, ['atGoalSignalId'], defaults.atGoalSignalId, signals),
    transitionCountSignalId: resolveSignalIdFromKeys(
      base,
      ['transitionCountSignalId'],
      defaults.transitionCountSignalId,
      signals
    ),
    lastTransitionSignalId: resolveSignalIdFromKeys(
      base,
      ['lastTransitionSignalId'],
      defaults.lastTransitionSignalId,
      signals
    ),
    availableStatesSignalId: resolveSignalIdFromKeys(
      base,
      ['availableStatesSignalId'],
      defaults.availableStatesSignalId,
      signals
    ),
    clearQueueCommandSignalId: resolveSignalIdFromKeys(
      base,
      ['clearQueueCommandSignalId', 'clearQueueSignalId'],
      defaults.clearQueueCommandSignalId,
      signals
    )
  };
}
