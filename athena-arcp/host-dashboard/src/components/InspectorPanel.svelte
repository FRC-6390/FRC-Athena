<script lang="ts">
  import { setContext } from 'svelte';
  import WidgetConfigEditor from './WidgetConfigEditor.svelte';
  import type { SignalRow } from '../lib/arcp';
  import { isActionSignal, isWritableSignal, signalRole } from '../lib/dashboard';
  import type { DashboardWidget, WidgetConfigRecord, WidgetKind } from '../lib/dashboard';

  type SignalMapRequest = {
    title: string;
    candidates: SignalRow[];
    selectedSignalId: number | null;
    allowNone: boolean;
    noneLabel: string;
    onPick: (signalId: number | null) => void;
  };

  type Props = {
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    selectedSignal: SignalRow | null;
    selectedWidget: DashboardWidget | null;
    widgetKinds: WidgetKind[];
    widgetKindLabel: (kind: WidgetKind) => string;
    onAddWidgetKind: (kind: WidgetKind) => void;
    onTriggerAction: (signalId: number) => void;
    onSelectTunableWidget: () => void;
    onRemoveWidget: (widgetId: string) => void;
    onUpdateWidgetTitle: (widgetId: string, title: string) => void;
    onUpdateWidgetConfig: (widgetId: string, config: WidgetConfigRecord | undefined) => void;
    onRequestSignalMap?: (request: SignalMapRequest) => void;
    showHeader?: boolean;
  };

  let {
    signals,
    signalById,
    selectedSignal,
    selectedWidget,
    widgetKinds,
    widgetKindLabel,
    onAddWidgetKind,
    onTriggerAction,
    onSelectTunableWidget,
    onRemoveWidget,
    onUpdateWidgetTitle,
    onUpdateWidgetConfig,
    onRequestSignalMap,
    showHeader = true
  }: Props = $props();

  let widgetTitleDraft = $state('');

  $effect(() => {
    widgetTitleDraft = selectedWidget?.title ?? '';
  });

  function commitWidgetTitle() {
    if (!selectedWidget) return;
    const next = widgetTitleDraft.trim();
    if (!next || next === selectedWidget.title) return;
    onUpdateWidgetTitle(selectedWidget.id, next);
  }

  setContext('arcp.signal-map-request', (request: SignalMapRequest) => onRequestSignalMap?.(request));
</script>

<aside class="inspector panel">
  {#if showHeader}
    <header>
      <h2>Inspector</h2>
      <p>Signal and panel details</p>
    </header>
  {/if}

  {#if selectedSignal}
    <dl>
      {#snippet infoRow(label: string, value: string, mono = false)}
        <dt>{label}</dt>
        <dd class:mono>{value}</dd>
      {/snippet}

      {@render infoRow('ID', String(selectedSignal.signal_id))}
      {@render infoRow('Role', signalRole(selectedSignal))}
      {@render infoRow('Type', selectedSignal.signal_type)}
      {@render infoRow('Path', selectedSignal.path, true)}
      {@render infoRow('Value', selectedSignal.value, true)}
    </dl>

    <section class="inspector-block">
      <h3>Create Widget</h3>
      <div class="kind-grid">
        {#each widgetKinds as kind}
          <button class="btn" onclick={() => onAddWidgetKind(kind)}>{widgetKindLabel(kind)}</button>
        {/each}
      </div>
    </section>

    <section class="inspector-block">
      <h3>Direct Control</h3>
      <div class="kind-grid">
        <button
          class="btn btn-danger"
          disabled={!isActionSignal(selectedSignal)}
          onclick={() => onTriggerAction(selectedSignal.signal_id)}
        >
          Trigger Action
        </button>
        <button class="btn" disabled={!isWritableSignal(selectedSignal)} onclick={onSelectTunableWidget}>
          Add Tunable Widget
        </button>
      </div>
    </section>
  {:else}
    <p>Select a signal from the explorer to inspect metadata and create panels.</p>
  {/if}

  {#if selectedWidget}
    <section class="inspector-block">
      <h3>Selected Widget</h3>
      <label>
        Header Text
        <div class="title-row">
          <input
            value={widgetTitleDraft}
            oninput={(event) => {
              widgetTitleDraft = (event.currentTarget as HTMLInputElement).value;
            }}
            onblur={commitWidgetTitle}
            onkeydown={(event) => {
              if (event.key !== 'Enter') return;
              event.preventDefault();
              commitWidgetTitle();
            }}
          />
          <button class="btn" onclick={commitWidgetTitle}>Set</button>
        </div>
      </label>
      <p>
        Grid: {selectedWidget.layout.w}x{selectedWidget.layout.h} at {selectedWidget.layout.x},
        {selectedWidget.layout.y}
      </p>
      <div class="kind-grid">
        <button class="btn btn-danger" onclick={() => onRemoveWidget(selectedWidget.id)}>Remove Widget</button>
      </div>
    </section>

    <WidgetConfigEditor
      {selectedWidget}
      {selectedSignal}
      {signals}
      {signalById}
      onUpdateConfig={onUpdateWidgetConfig}
    />
  {/if}
</aside>

<style>
  .inspector {
    padding: 0.66rem;
    display: grid;
    grid-auto-rows: max-content;
    gap: 0.58rem;
    height: 100%;
    min-height: 0;
    overflow: auto;
  }

  h2 {
    margin: 0;
    font-size: 0.88rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.18rem 0 0;
    font-size: 0.73rem;
    color: var(--text-soft);
  }

  dl {
    margin: 0;
    padding: 0.56rem;
    border-radius: 8px;
    border: 1px solid var(--border-subtle);
    background: var(--surface-2);
    display: grid;
    grid-template-columns: 68px 1fr;
    gap: 0.28rem 0.42rem;
  }

  dt {
    color: var(--text-soft);
    font-size: 0.74rem;
  }

  dd {
    margin: 0;
    font-size: 0.78rem;
    word-break: break-word;
    color: var(--text);
  }

  .mono {
    font-family: var(--font-mono);
  }

  .inspector > p {
    color: var(--text-soft);
    margin: 0;
    font-size: 0.8rem;
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.6rem;
  }

  .inspector-block {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.58rem;
    display: grid;
    gap: 0.4rem;
  }

  .inspector-block h3 {
    margin: 0;
    font-size: 0.78rem;
    color: var(--text-strong);
  }

  .inspector-block p {
    margin: 0;
    font-size: 0.76rem;
    color: var(--text-soft);
  }

  .inspector-block label {
    display: grid;
    gap: 0.2rem;
    color: var(--text-soft);
    font-size: 0.68rem;
  }

  .inspector-block input {
    width: 100%;
    min-width: 0;
    padding: 0.24rem 0.34rem;
    font-size: 0.7rem;
  }

  .kind-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.35rem;
  }

  .title-row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.3rem;
    align-items: center;
  }
</style>
