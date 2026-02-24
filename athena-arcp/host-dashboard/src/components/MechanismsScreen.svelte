<script lang="ts">
  import type { SignalRow } from '../lib/arcp';

  type Props = {
    signals: SignalRow[];
    selectedId: number | null;
    onSelectSignal: (signalId: number) => void;
    onTriggerAction: (signalId: number) => void;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  type DeviceGroup = {
    key: string;
    label: string;
    signals: SignalRow[];
  };

  type MechanismGroup = {
    key: string;
    label: string;
    signals: SignalRow[];
    devices: DeviceGroup[];
  };

  let { signals, selectedId, onSelectSignal, onTriggerAction, onSendSet }: Props = $props();

  let selectedMechanismKey = $state('');
  let selectedDeviceKey = $state('');
  let query = $state('');
  let draftBySignal = $state<Record<number, string>>({});

  function splitPath(path: string): string[] {
    return path
      .split('/')
      .map((part) => part.trim())
      .filter((part) => part.length > 0);
  }

  function toLabel(token: string): string {
    const cleaned = token.replace(/[_-]+/g, ' ').trim();
    if (!cleaned) return 'Unknown';
    return cleaned
      .split(' ')
      .filter(Boolean)
      .map((piece) => piece[0].toUpperCase() + piece.slice(1))
      .join(' ');
  }

  function leaf(path: string): string {
    const parts = splitPath(path);
    return parts[parts.length - 1] ?? path;
  }

  function classify(signal: SignalRow): { mechanismKey: string; deviceKey: string } {
    const parts = splitPath(signal.path);
    if (parts.length === 0) {
      return { mechanismKey: 'system', deviceKey: 'general' };
    }

    let mechanismKey = parts[0] ?? 'system';
    let deviceKey = parts[1] ?? 'general';

    const mechIndex = parts.indexOf('mechanisms');
    if (mechIndex >= 0 && parts[mechIndex + 1]) {
      mechanismKey = parts[mechIndex + 1];
      deviceKey = parts[mechIndex + 2] ?? 'general';
      return { mechanismKey, deviceKey };
    }

    const deviceIndex = parts.indexOf('devices');
    if (deviceIndex >= 0 && parts[deviceIndex + 1]) {
      mechanismKey = parts[Math.max(0, deviceIndex - 1)] ?? mechanismKey;
      deviceKey = parts[deviceIndex + 1] ?? 'general';
      return { mechanismKey, deviceKey };
    }

    if ((mechanismKey === 'robot' || mechanismKey === 'athena') && parts[1]) {
      mechanismKey = parts[1];
      deviceKey = parts[2] ?? 'general';
    }

    return { mechanismKey, deviceKey };
  }

  const filteredSignals = $derived.by(() => {
    const q = query.trim().toLowerCase();
    if (!q) return signals;

    return signals.filter((signal) => {
      return (
        signal.path.toLowerCase().includes(q) ||
        signal.signal_type.toLowerCase().includes(q) ||
        signal.access.toLowerCase().includes(q) ||
        signal.kind.toLowerCase().includes(q) ||
        String(signal.signal_id).includes(q)
      );
    });
  });

  const mechanisms = $derived.by(() => {
    const map = new Map<string, { label: string; signals: SignalRow[]; devices: Map<string, SignalRow[]> }>();

    for (const signal of filteredSignals) {
      const { mechanismKey, deviceKey } = classify(signal);
      if (!map.has(mechanismKey)) {
        map.set(mechanismKey, {
          label: toLabel(mechanismKey),
          signals: [],
          devices: new Map<string, SignalRow[]>()
        });
      }

      const mech = map.get(mechanismKey)!;
      mech.signals.push(signal);
      if (!mech.devices.has(deviceKey)) {
        mech.devices.set(deviceKey, []);
      }
      mech.devices.get(deviceKey)!.push(signal);
    }

    return [...map.entries()]
      .map(([key, value]): MechanismGroup => {
        const devices: DeviceGroup[] = [...value.devices.entries()]
          .map(([deviceKey, deviceSignals]) => ({
            key: deviceKey,
            label: toLabel(deviceKey),
            signals: [...deviceSignals].sort((a, b) => a.path.localeCompare(b.path))
          }))
          .sort((a, b) => a.label.localeCompare(b.label));

        return {
          key,
          label: value.label,
          signals: [...value.signals].sort((a, b) => a.path.localeCompare(b.path)),
          devices
        };
      })
      .sort((a, b) => a.label.localeCompare(b.label));
  });

  const selectedMechanism = $derived(
    mechanisms.find((entry) => entry.key === selectedMechanismKey) ?? null
  );

  const selectedDevice = $derived(
    selectedMechanism?.devices.find((entry) => entry.key === selectedDeviceKey) ?? null
  );

  const deviceSignals = $derived(selectedDevice?.signals ?? []);

  $effect(() => {
    if (mechanisms.length === 0) {
      selectedMechanismKey = '';
      selectedDeviceKey = '';
      return;
    }

    if (!mechanisms.some((entry) => entry.key === selectedMechanismKey)) {
      selectedMechanismKey = mechanisms[0].key;
    }
  });

  $effect(() => {
    if (!selectedMechanism || selectedMechanism.devices.length === 0) {
      selectedDeviceKey = '';
      return;
    }

    if (!selectedMechanism.devices.some((entry) => entry.key === selectedDeviceKey)) {
      selectedDeviceKey = selectedMechanism.devices[0].key;
    }
  });

  $effect(() => {
    const next = { ...draftBySignal };
    let changed = false;
    const writableIds = new Set<number>();

    for (const signal of deviceSignals) {
      if (signal.access !== 'write') continue;
      writableIds.add(signal.signal_id);
      if (next[signal.signal_id] === undefined) {
        next[signal.signal_id] = signal.value === '-' ? '' : signal.value;
        changed = true;
      }
    }

    for (const id of Object.keys(next).map(Number)) {
      if (!writableIds.has(id)) {
        delete next[id];
        changed = true;
      }
    }

    if (changed) {
      draftBySignal = next;
    }
  });

  function isNumeric(signal: SignalRow): boolean {
    return signal.signal_type === 'f64' || signal.signal_type === 'i64';
  }

  function isBoolean(signal: SignalRow): boolean {
    return signal.signal_type === 'bool';
  }

  function enumOptions(signal: SignalRow): string[] {
    const options: string[] = [];
    const name = leaf(signal.path).toLowerCase();
    const value = signal.value.trim().toLowerCase();

    if (isBoolean(signal)) {
      return ['true', 'false'];
    }

    if (
      name.includes('neutral') ||
      name.includes('brake') ||
      name.includes('coast') ||
      name.includes('idlemode')
    ) {
      options.push('coast', 'brake');
    }

    if (name.includes('invert') || name.includes('enabled') || name.includes('enable')) {
      options.push('true', 'false');
    }

    if (signal.value.includes('|') || signal.value.includes(',')) {
      const parsed = signal.value
        .split(/[|,]/)
        .map((entry) => entry.trim())
        .filter((entry) => entry.length > 0);
      options.push(...parsed);
    }

    if (value === 'coast' || value === 'brake') {
      options.push('coast', 'brake');
    }

    return [...new Set(options.map((entry) => entry.trim()).filter((entry) => entry.length > 0))];
  }

  function sliderSpec(signal: SignalRow): { min: number; max: number; step: number } {
    const name = leaf(signal.path).toLowerCase();
    const current = Number(signal.value);
    const value = Number.isFinite(current) ? current : 0;

    if (name.includes('speed') || name.includes('output') || name.includes('percent')) {
      return { min: -1, max: 1, step: 0.01 };
    }

    if (name.includes('voltage')) {
      return { min: -12, max: 12, step: 0.1 };
    }

    if (name.includes('ratio')) {
      return { min: 0, max: 50, step: 0.01 };
    }

    if (name.includes('offset') || name.includes('angle') || name.includes('heading')) {
      return { min: -360, max: 360, step: 0.1 };
    }

    const span = Math.max(1, Math.abs(value) * 1.5);
    return {
      min: value - span,
      max: value + span,
      step: Math.max(0.001, span / 200)
    };
  }

  function setDraft(signalId: number, value: string) {
    draftBySignal = {
      ...draftBySignal,
      [signalId]: value
    };
  }

  function applyDraft(signal: SignalRow) {
    const value = draftBySignal[signal.signal_id] ?? '';
    onSendSet(signal.signal_id, value);
  }

  const deviceSummary = $derived.by(() => {
    const total = deviceSignals.length;
    const writable = deviceSignals.filter((signal) => signal.access === 'write').length;
    const invoke = deviceSignals.filter((signal) => signal.access === 'invoke').length;
    return { total, writable, invoke };
  });
</script>

<section class="mechanisms-screen panel">
  <header>
    <h2>Mechanisms & Devices</h2>
    <p>Browse grouped mechanism data and control writable/invoke signals per device.</p>
  </header>

  <div class="toolbar">
    <input
      value={query}
      placeholder="Filter by path/type/access"
      oninput={(event) => (query = (event.currentTarget as HTMLInputElement).value)}
    />
    <span class="summary-chip">{mechanisms.length} mechanisms</span>
  </div>

  <div class="body">
    <aside class="column mechanisms-list">
      <h3>Mechanisms</h3>
      <div class="scroll">
        {#if mechanisms.length === 0}
          <p class="empty">No mechanisms found.</p>
        {:else}
          {#each mechanisms as mechanism (mechanism.key)}
            <button
              class={`item ${selectedMechanismKey === mechanism.key ? 'active' : ''}`}
              onclick={() => {
                selectedMechanismKey = mechanism.key;
                selectedDeviceKey = mechanism.devices[0]?.key ?? '';
              }}
            >
              <strong>{mechanism.label}</strong>
              <span>{mechanism.devices.length} devices · {mechanism.signals.length} signals</span>
            </button>
          {/each}
        {/if}
      </div>
    </aside>

    <aside class="column devices-list">
      <h3>Devices</h3>
      <div class="scroll">
        {#if !selectedMechanism || selectedMechanism.devices.length === 0}
          <p class="empty">No devices found for this mechanism.</p>
        {:else}
          {#each selectedMechanism.devices as device (device.key)}
            <button
              class={`item ${selectedDeviceKey === device.key ? 'active' : ''}`}
              onclick={() => (selectedDeviceKey = device.key)}
            >
              <strong>{device.label}</strong>
              <span>{device.signals.length} signals</span>
            </button>
          {/each}
        {/if}
      </div>
    </aside>

    <section class="column device-view">
      {#if selectedDevice}
        <div class="device-head">
          <h3>{selectedDevice.label}</h3>
          <div class="chips">
            <span>{deviceSummary.total} total</span>
            <span>{deviceSummary.writable} writable</span>
            <span>{deviceSummary.invoke} actions</span>
          </div>
        </div>

        <div class="signal-list">
          {#each deviceSignals as signal (signal.signal_id)}
            {@const options = enumOptions(signal)}
            {@const slider = sliderSpec(signal)}
            <article class={`signal-card ${selectedId === signal.signal_id ? 'selected' : ''}`}>
              <button class="signal-meta" onclick={() => onSelectSignal(signal.signal_id)}>
                <strong>{leaf(signal.path)}</strong>
                <span>#{signal.signal_id} · {signal.signal_type} · {signal.access}</span>
                <code>{signal.path}</code>
                <em>Current: {signal.value}</em>
              </button>

              {#if signal.access === 'invoke'}
                <div class="controls">
                  <button class="btn btn-danger" onclick={() => onTriggerAction(signal.signal_id)}>Trigger</button>
                </div>
              {:else if signal.access === 'write'}
                <div class="controls">
                  {#if options.length > 0}
                    <select
                      value={draftBySignal[signal.signal_id] ?? signal.value}
                      onchange={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLSelectElement).value)}
                    >
                      {#each options as option}
                        <option value={option}>{option}</option>
                      {/each}
                    </select>
                    <button class="btn btn-primary" onclick={() => applyDraft(signal)}>Set</button>
                  {:else if isNumeric(signal)}
                    <input
                      type="range"
                      min={slider.min}
                      max={slider.max}
                      step={slider.step}
                      value={Number(draftBySignal[signal.signal_id] ?? signal.value) || 0}
                      oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).value)}
                    />
                    <input
                      value={draftBySignal[signal.signal_id] ?? signal.value}
                      oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).value)}
                    />
                    <button class="btn btn-primary" onclick={() => applyDraft(signal)}>Set</button>
                  {:else if isBoolean(signal)}
                    <label class="toggle-row">
                      <input
                        type="checkbox"
                        checked={(draftBySignal[signal.signal_id] ?? signal.value) === 'true'}
                        onchange={(event) =>
                          setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).checked ? 'true' : 'false')}
                      />
                      Toggle
                    </label>
                    <button class="btn btn-primary" onclick={() => applyDraft(signal)}>Set</button>
                  {:else}
                    <input
                      value={draftBySignal[signal.signal_id] ?? signal.value}
                      oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).value)}
                    />
                    <button class="btn btn-primary" onclick={() => applyDraft(signal)}>Set</button>
                  {/if}
                </div>
              {/if}
            </article>
          {/each}
        </div>
      {:else}
        <p class="empty">Select a mechanism and device to view controls.</p>
      {/if}
    </section>
  </div>
</section>

<style>
  .mechanisms-screen {
    padding: 0.7rem;
    display: grid;
    grid-template-rows: auto auto 1fr;
    gap: 0.58rem;
    min-height: 0;
  }

  header h2 {
    margin: 0;
    font-size: 0.98rem;
    color: var(--text-strong);
  }

  header p {
    margin: 0.2rem 0 0;
    color: var(--text-soft);
    font-size: 0.76rem;
  }

  .toolbar {
    display: flex;
    gap: 0.45rem;
    align-items: center;
  }

  .toolbar input {
    flex: 1 1 auto;
    min-width: 0;
  }

  .summary-chip {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.22rem 0.48rem;
    font-size: 0.7rem;
    color: var(--text-soft);
    background: var(--surface-3);
    white-space: nowrap;
  }

  .body {
    min-height: 0;
    display: grid;
    grid-template-columns: minmax(210px, 260px) minmax(220px, 300px) minmax(0, 1fr);
    gap: 0.54rem;
  }

  .column {
    border: 1px solid var(--border-subtle);
    border-radius: 10px;
    background: var(--surface-2);
    padding: 0.48rem;
    min-height: 0;
    display: grid;
    grid-template-rows: auto 1fr;
    gap: 0.34rem;
  }

  h3 {
    margin: 0;
    font-size: 0.79rem;
    color: var(--text-strong);
  }

  .scroll {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.28rem;
    align-content: start;
  }

  .item {
    width: 100%;
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: var(--surface-3);
    color: var(--text);
    padding: 0.34rem 0.42rem;
    text-align: left;
    display: grid;
    gap: 0.16rem;
  }

  .item strong {
    font-size: 0.74rem;
    color: var(--text-strong);
    line-height: 1.1;
  }

  .item span {
    font-size: 0.66rem;
    color: var(--text-soft);
  }

  .item.active {
    border-color: rgba(180, 35, 45, 0.64);
    background: rgba(180, 35, 45, 0.2);
  }

  .device-view {
    grid-template-rows: auto 1fr;
  }

  .device-head {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.5rem;
  }

  .chips {
    display: inline-flex;
    gap: 0.24rem;
    flex-wrap: wrap;
    justify-content: flex-end;
  }

  .chips span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.14rem 0.38rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    background: var(--surface-3);
    white-space: nowrap;
  }

  .signal-list {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.34rem;
    align-content: start;
  }

  .signal-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface);
    padding: 0.44rem;
    display: grid;
    gap: 0.36rem;
  }

  .signal-card.selected {
    border-color: rgba(180, 35, 45, 0.64);
    box-shadow: inset 0 0 0 1px rgba(180, 35, 45, 0.26);
  }

  .signal-meta {
    border: 0;
    background: transparent;
    color: inherit;
    text-align: left;
    padding: 0;
    display: grid;
    gap: 0.12rem;
  }

  .signal-meta strong {
    font-size: 0.78rem;
    color: var(--text-strong);
  }

  .signal-meta span {
    font-size: 0.67rem;
    color: var(--text-soft);
  }

  .signal-meta code {
    font-size: 0.63rem;
    color: var(--text-soft);
    font-family: var(--font-mono);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .signal-meta em {
    font-size: 0.68rem;
    color: var(--text);
    font-style: normal;
    font-family: var(--font-mono);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .controls {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.28rem;
    align-items: center;
  }

  .controls input,
  .controls select {
    width: 100%;
    min-width: 0;
    padding: 0.28rem 0.36rem;
    font-size: 0.7rem;
  }

  .controls input[type='range'] {
    grid-column: 1 / -1;
    padding: 0;
  }

  .controls .btn {
    padding: 0.26rem 0.48rem;
    font-size: 0.66rem;
  }

  .toggle-row {
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
    font-size: 0.7rem;
    color: var(--text-soft);
  }

  .toggle-row input {
    width: auto;
    margin: 0;
  }

  .empty {
    margin: 0;
    border: 1px dashed var(--border-emphasis);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.62rem;
    color: var(--text-soft);
    font-size: 0.76rem;
  }

  @media (max-width: 1300px) {
    .body {
      grid-template-columns: minmax(200px, 240px) minmax(210px, 250px) minmax(0, 1fr);
    }
  }

  @media (max-width: 1080px) {
    .body {
      grid-template-columns: 1fr;
      grid-template-rows: minmax(140px, 26vh) minmax(140px, 26vh) minmax(0, 1fr);
    }
  }
</style>
