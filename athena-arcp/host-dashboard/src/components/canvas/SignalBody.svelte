<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { DashboardWidget } from '../../lib/dashboard';
  import { classicComponentFor } from '../widget-registry';
  import type { CanvasRuntime } from './types';

  let {
    widget,
    signal,
    unavailableHint,
    runtime
  }: {
    widget: DashboardWidget;
    signal: SignalRow | null;
    unavailableHint: string;
    runtime: CanvasRuntime;
  } = $props();

  const Body = $derived(classicComponentFor(widget.kind));
</script>

{#if signal}
  <Body
    {signal}
    signals={runtime.signals}
    signalById={runtime.signalById}
    widgetId={widget.id}
    configRaw={widget.config}
    historyBySignal={runtime.historyBySignal}
    historyTimeBySignal={runtime.historyTimeBySignal}
    nowMs={runtime.nowMs}
    inputValue={runtime.widgetInputs[widget.id]}
    onInput={(value) => runtime.onWidgetInput(widget.id, value)}
    onSendSet={runtime.onSendSet}
    onSendAction={runtime.onSendAction}
  />
{:else}
  <div class="signal-unavailable">
    <strong>Signal unavailable</strong>
    <span class="signal-unavailable-hint">{unavailableHint}</span>
    <small>Reconnect ARCP or remap this widget in Inspector.</small>
  </div>
{/if}

<style>
  .signal-unavailable {
    display: grid;
    gap: 0.18rem;
    border: 1px dashed rgba(153, 164, 180, 0.4);
    border-radius: 6px;
    padding: 0.34rem 0.42rem;
    background:
      linear-gradient(180deg, rgba(69, 79, 97, 0.3), rgba(36, 43, 56, 0.24)),
      repeating-linear-gradient(
        135deg,
        rgba(153, 164, 180, 0.08),
        rgba(153, 164, 180, 0.08) 6px,
        transparent 6px,
        transparent 12px
      );
  }

  .signal-unavailable strong {
    font-size: 0.7rem;
    line-height: 1.15;
    color: var(--text-strong);
  }

  .signal-unavailable-hint {
    font-family: var(--font-mono);
    font-size: 0.64rem;
    color: var(--text-soft);
    overflow-wrap: anywhere;
  }

  .signal-unavailable small {
    font-size: 0.62rem;
    line-height: 1.2;
    color: color-mix(in srgb, var(--text-soft) 86%, white 14%);
  }
</style>
