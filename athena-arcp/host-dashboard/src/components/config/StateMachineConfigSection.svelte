<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import SignalMapField from '../SignalMapField.svelte';
  import { readStateMachineConfig } from '../../lib/widget-config';
  import type { StateMachineWidgetConfig } from '../../lib/widget-config';
  import type { SignalSectionProps } from './types';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const boolSignals = $derived(pools.boolSignals);
  const stringSignals = $derived(pools.stringSignals);
  const actionSignals = $derived(pools.actionSignals);

  function defaultSignalIdFor(candidates: SignalRow[]): number | null {
    if (selectedSignal && candidates.some((entry) => entry.signal_id === selectedSignal.signal_id)) {
      return selectedSignal.signal_id;
    }
    return candidates[0]?.signal_id ?? null;
  }
  function toggleSignalMapping(
    currentSignalId: number | null,
    enabled: boolean,
    candidates: SignalRow[]
  ): number | null {
    if (!enabled) return null;
    return currentSignalId ?? defaultSignalIdFor(candidates);
  }
</script>

{#if selectedWidget.kind === 'state_machine'}
  {@const stateMachine = readStateMachineConfig(selectedWidget.config, primarySignal, signals)}
  {@const mappedCount = [
    stateMachine.currentStateSignalId,
    stateMachine.goalStateSignalId,
    stateMachine.nextStateSignalId,
    stateMachine.queueSignalId,
    stateMachine.atGoalSignalId,
    stateMachine.transitionCountSignalId,
    stateMachine.lastTransitionSignalId,
    stateMachine.availableStatesSignalId,
    stateMachine.clearQueueCommandSignalId
  ].filter((id) => id !== null).length}
  <section class="config-block">
    <h4>State Machine</h4>
    <p class="hint">Maps state telemetry, writable toggles, and command actions into one panel.</p>
    <button
      class="btn"
      onclick={() => {
        const seed = selectedSignal ?? primarySignal;
        setConfig(readStateMachineConfig(undefined, seed, signals));
      }}
    >
      Auto remap from selected signal
    </button>
    <div class="chips">
      <span>{mappedCount} mapped fields</span>
      {#if stateMachine.goalStateSignalId !== null}<span>goal setpoint</span>{/if}
      {#if stateMachine.clearQueueCommandSignalId !== null}<span>clear queue</span>{/if}
    </div>
    <div class="row-grid">
      <SignalMapField
        label="Current state signal"
        selectedSignalId={stateMachine.currentStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.currentStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            currentStateSignalId: toggleSignalMapping(stateMachine.currentStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, currentStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Goal state signal"
        selectedSignalId={stateMachine.goalStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.goalStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            goalStateSignalId: toggleSignalMapping(stateMachine.goalStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, goalStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Next state signal"
        selectedSignalId={stateMachine.nextStateSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.nextStateSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            nextStateSignalId: toggleSignalMapping(stateMachine.nextStateSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, nextStateSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Queue signal"
        selectedSignalId={stateMachine.queueSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.queueSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            queueSignalId: toggleSignalMapping(stateMachine.queueSignalId, enabled, stringSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, queueSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="At-goal signal"
        selectedSignalId={stateMachine.atGoalSignalId}
        candidates={boolSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.atGoalSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            atGoalSignalId: toggleSignalMapping(stateMachine.atGoalSignalId, enabled, boolSignals)
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) => setConfig({ ...stateMachine, atGoalSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Transition-count signal"
        selectedSignalId={stateMachine.transitionCountSignalId}
        candidates={numericSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.transitionCountSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            transitionCountSignalId: toggleSignalMapping(
              stateMachine.transitionCountSignalId,
              enabled,
              numericSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, transitionCountSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Last-transition signal"
        selectedSignalId={stateMachine.lastTransitionSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.lastTransitionSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            lastTransitionSignalId: toggleSignalMapping(
              stateMachine.lastTransitionSignalId,
              enabled,
              stringSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, lastTransitionSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Available-states signal"
        selectedSignalId={stateMachine.availableStatesSignalId}
        candidates={stringSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.availableStatesSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            availableStatesSignalId: toggleSignalMapping(
              stateMachine.availableStatesSignalId,
              enabled,
              stringSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, availableStatesSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
      <SignalMapField
        label="Clear-queue action"
        selectedSignalId={stateMachine.clearQueueCommandSignalId}
        candidates={actionSignals}
        {signalById}
        optional={true}
        enabled={stateMachine.clearQueueCommandSignalId !== null}
        onToggleEnabled={(enabled) =>
          setConfig({
            ...stateMachine,
            clearQueueCommandSignalId: toggleSignalMapping(
              stateMachine.clearQueueCommandSignalId,
              enabled,
              actionSignals
            )
          } satisfies StateMachineWidgetConfig)}
        onChange={(signalId) =>
          setConfig({ ...stateMachine, clearQueueCommandSignalId: signalId } satisfies StateMachineWidgetConfig)}
      />
    </div>
  </section>
{/if}
