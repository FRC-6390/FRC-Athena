<script lang="ts">
  import type { SignalRow } from '../lib/arcp';
  import { isLayoutWidgetKind, type DashboardWidget, type WidgetConfigRecord } from '../lib/dashboard';
  import { buildEditorPools } from '../lib/widget-config/editor-pools';
  import { LAYOUT_CONFIG_SECTIONS, SIGNAL_CONFIG_SECTIONS } from './config';
  import './config/config-editor.css';

  type Props = {
    selectedWidget: DashboardWidget;
    selectedSignal: SignalRow | null;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    onUpdateConfig: (widgetId: string, config: WidgetConfigRecord | undefined) => void;
  };

  let { selectedWidget, selectedSignal, signals, signalById, onUpdateConfig }: Props = $props();

  /** Signal the widget edits: its bound signal, else the selection, else the first signal. */
  const primarySignal = $derived(
    signalById.get(selectedWidget.signalId) ?? selectedSignal ?? signals[0] ?? null
  );
  const pools = $derived(buildEditorPools(signals));
  const signalSection = $derived(SIGNAL_CONFIG_SECTIONS[selectedWidget.kind as keyof typeof SIGNAL_CONFIG_SECTIONS]);
  const layoutSection = $derived(LAYOUT_CONFIG_SECTIONS[selectedWidget.kind]);

  function setConfig(config: WidgetConfigRecord | undefined) {
    onUpdateConfig(selectedWidget.id, config);
  }
</script>

<div class="config-editor">
  {#if !primarySignal && !isLayoutWidgetKind(selectedWidget.kind)}
    <p class="empty">Config unavailable: no signal bound to this widget.</p>
  {:else if primarySignal && signalSection}
    {@const Section = signalSection}
    <Section {selectedWidget} {primarySignal} {selectedSignal} {signals} {signalById} {pools} {setConfig} />
  {:else if layoutSection}
    {@const Layout = layoutSection}
    <Layout {selectedWidget} {setConfig} />
  {:else}
    <p class="empty">This widget has no extra configuration.</p>
  {/if}
</div>
