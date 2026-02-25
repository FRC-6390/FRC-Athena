<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readDioConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let outputDraft = $state(false);
  let invertedDraft = $state(false);
  let channelDraft = $state('0');
  let portDraft = $state('0');
  let modeDraft = $state('');
  let outputSeedSignalId = $state<number | null>(null);
  let invertedSeedSignalId = $state<number | null>(null);
  let channelSeedSignalId = $state<number | null>(null);
  let portSeedSignalId = $state<number | null>(null);
  let modeSeedSignalId = $state<number | null>(null);
  let extraDraftBySignal = $state<Record<number, string>>({});

  const config = $derived(readDioConfig(configRaw, signal, signals));

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

  function rowFor(signalId: number | null): SignalRow | null {
    if (signalId === null) return null;
    return signalById.get(signalId) ?? null;
  }

  function boolFor(signalId: number | null): boolean | null {
    const row = rowFor(signalId);
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

  function numberFor(signalId: number | null): number | null {
    const row = rowFor(signalId);
    if (!row) return null;
    const parsed = Number(row.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function textFor(signalId: number | null): string | null {
    const row = rowFor(signalId);
    if (!row) return null;
    return row.value;
  }

  function isWritable(row: SignalRow | null): row is SignalRow {
    return !!row && row.access === 'write';
  }

  function isDigitalBool(row: SignalRow | null): row is SignalRow {
    return !!row && row.signal_type === 'bool';
  }

  function formatNumber(value: number | null): string {
    if (value === null) return '--';
    return String(Math.round(value));
  }

  function formatValue(raw: string): string {
    if (raw === '-') return '--';
    return raw;
  }

  function parseNumericDraft(raw: string): number | null {
    const parsed = Number(raw.trim());
    return Number.isFinite(parsed) ? parsed : null;
  }

  function sendRaw(row: SignalRow, valueRaw: string) {
    onSendSet(row.signal_id, valueRaw);
  }

  function extraInputKind(row: SignalRow): 'bool' | 'number' | 'text' {
    if (row.signal_type === 'bool') return 'bool';
    if (row.signal_type === 'f64' || row.signal_type === 'i64') return 'number';
    return 'text';
  }

  const dioParentPath = $derived(parentPath(signal.path));

  const siblingSignals = $derived.by(() =>
    signals
      .filter((entry) => parentPath(entry.path) === dioParentPath)
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const valueRow = $derived(rowFor(config.valueSignalId));
  const outputRow = $derived(rowFor(config.outputSignalId));
  const invertedRow = $derived(rowFor(config.invertedSignalId));
  const channelRow = $derived(rowFor(config.channelSignalId));
  const portRow = $derived(rowFor(config.portSignalId));
  const modeRow = $derived(rowFor(config.modeSignalId));
  const nameRow = $derived(rowFor(config.nameSignalId));
  const typeRow = $derived(rowFor(config.typeSignalId));

  const knownSignalIds = $derived.by(() => {
    const ids = [
      config.valueSignalId,
      config.outputSignalId,
      config.invertedSignalId,
      config.channelSignalId,
      config.portSignalId,
      config.modeSignalId,
      config.nameSignalId,
      config.typeSignalId
    ];
    const next = new Set<number>();
    for (const id of ids) {
      if (typeof id === 'number' && id > 0) next.add(id);
    }
    return next;
  });

  const extraSignals = $derived.by(() =>
    siblingSignals.filter((entry) => !knownSignalIds.has(entry.signal_id))
  );

  const showExtraFieldsSection = $derived(config.showOtherFields && extraSignals.length > 0);

  const valueState = $derived.by(() => {
    const direct = boolFor(config.valueSignalId);
    if (direct !== null) return direct;
    return boolFor(config.outputSignalId);
  });

  const outputWritable = $derived(isWritable(outputRow) && isDigitalBool(outputRow));
  const invertedWritable = $derived(isWritable(invertedRow) && isDigitalBool(invertedRow));
  const channelWritable = $derived(isWritable(channelRow));
  const portWritable = $derived(isWritable(portRow));
  const modeWritable = $derived(isWritable(modeRow));

  const channelValue = $derived(numberFor(config.channelSignalId));
  const portValue = $derived(numberFor(config.portSignalId));
  const modeValue = $derived(textFor(config.modeSignalId));
  const typeValue = $derived(textFor(config.typeSignalId));
  const nameValue = $derived(textFor(config.nameSignalId));

  $effect(() => {
    if (!outputWritable || !outputRow) {
      outputSeedSignalId = null;
      return;
    }
    if (outputSeedSignalId === outputRow.signal_id) return;
    outputSeedSignalId = outputRow.signal_id;
    outputDraft = boolFor(outputRow.signal_id) ?? false;
  });

  $effect(() => {
    if (!invertedWritable || !invertedRow) {
      invertedSeedSignalId = null;
      return;
    }
    if (invertedSeedSignalId === invertedRow.signal_id) return;
    invertedSeedSignalId = invertedRow.signal_id;
    invertedDraft = boolFor(invertedRow.signal_id) ?? false;
  });

  $effect(() => {
    if (!channelWritable || !channelRow) {
      channelSeedSignalId = null;
      return;
    }
    if (channelSeedSignalId === channelRow.signal_id) return;
    channelSeedSignalId = channelRow.signal_id;
    channelDraft = String(Math.round(channelValue ?? 0));
  });

  $effect(() => {
    if (!portWritable || !portRow) {
      portSeedSignalId = null;
      return;
    }
    if (portSeedSignalId === portRow.signal_id) return;
    portSeedSignalId = portRow.signal_id;
    portDraft = String(Math.round(portValue ?? 0));
  });

  $effect(() => {
    if (!modeWritable || !modeRow) {
      modeSeedSignalId = null;
      return;
    }
    if (modeSeedSignalId === modeRow.signal_id) return;
    modeSeedSignalId = modeRow.signal_id;
    modeDraft = modeValue && modeValue !== '-' ? modeValue : '';
  });

  $effect(() => {
    const next = { ...extraDraftBySignal };
    let changed = false;
    const writableIds = new Set<number>();

    for (const row of extraSignals) {
      if (row.access !== 'write') continue;
      writableIds.add(row.signal_id);
      if (next[row.signal_id] === undefined) {
        next[row.signal_id] = row.value === '-' ? '' : row.value;
        changed = true;
      }
    }

    for (const key of Object.keys(next)) {
      const signalId = Number(key);
      if (!writableIds.has(signalId)) {
        delete next[signalId];
        changed = true;
      }
    }

    if (changed) {
      extraDraftBySignal = next;
    }
  });

  function sendOutput() {
    if (!outputRow || !outputWritable) return;
    sendRaw(outputRow, outputDraft ? 'true' : 'false');
  }

  function sendInverted() {
    if (!invertedRow || !invertedWritable) return;
    sendRaw(invertedRow, invertedDraft ? 'true' : 'false');
  }

  function sendChannel() {
    if (!channelRow || !channelWritable) return;
    const parsed = parseNumericDraft(channelDraft);
    if (parsed === null) return;
    sendRaw(channelRow, String(Math.round(parsed)));
  }

  function sendPort() {
    if (!portRow || !portWritable) return;
    const parsed = parseNumericDraft(portDraft);
    if (parsed === null) return;
    sendRaw(portRow, String(Math.round(parsed)));
  }

  function sendMode() {
    if (!modeRow || !modeWritable) return;
    if (modeRow.signal_type === 'bool') {
      const normalized = modeDraft.trim().toLowerCase();
      if (normalized !== 'true' && normalized !== 'false') return;
      sendRaw(modeRow, normalized);
      return;
    }
    if (!modeDraft.trim()) return;
    sendRaw(modeRow, modeDraft.trim());
  }

  function setExtraDraft(signalId: number, value: string) {
    extraDraftBySignal = {
      ...extraDraftBySignal,
      [signalId]: value
    };
  }

  function applyExtra(row: SignalRow) {
    if (row.access !== 'write') return;
    const raw = extraDraftBySignal[row.signal_id] ?? '';
    const kind = extraInputKind(row);
    if (kind === 'number') {
      const parsed = parseNumericDraft(raw);
      if (parsed === null) return;
      sendRaw(row, String(parsed));
      return;
    }
    if (kind === 'bool') {
      const normalized = raw.trim().toLowerCase();
      if (normalized !== 'true' && normalized !== 'false') return;
      sendRaw(row, normalized);
      return;
    }
    if (!raw.trim()) return;
    sendRaw(row, raw);
  }
</script>

<div class={`dio-root ${showExtraFieldsSection ? '' : 'no-extra-fields'}`}>
  <section class="identity">
    <div class="chip-row">
      <span class={`chip ${valueState ? 'ok' : valueState === false ? 'bad' : ''}`}>
        {valueState === null ? 'unknown' : valueState ? 'active' : 'inactive'}
      </span>
      {#if boolFor(config.invertedSignalId) !== null}
        <span class="chip">{boolFor(config.invertedSignalId) ? 'inverted' : 'normal'}</span>
      {/if}
      {#if modeValue && modeValue !== '-'}
        <span class="chip">{modeValue}</span>
      {/if}
      {#if typeValue && typeValue !== '-'}
        <span class="chip">{typeValue}</span>
      {/if}
    </div>

    <div class="identity-grid">
      <div><span>Name</span><strong>{nameValue && nameValue !== '-' ? nameValue : leafPath(signal.path)}</strong></div>
      <div><span>Channel</span><strong>{formatNumber(channelValue)}</strong></div>
      <div><span>Port</span><strong>{formatNumber(portValue)}</strong></div>
      <div><span>Path</span><strong class="path">{dioParentPath || signal.path}</strong></div>
    </div>
  </section>

  <section class="metric-grid">
    <article class="metric-card">
      <header><span>Value</span><strong>{valueState === null ? '--' : valueState ? 'true' : 'false'}</strong></header>
      <div class="bar"><span style={`width:${valueState ? '100' : '0'}%;`}></span></div>
    </article>
    <article class="metric-card compact">
      <header><span>Inverted</span><strong>{boolFor(config.invertedSignalId) === null ? '--' : boolFor(config.invertedSignalId) ? 'true' : 'false'}</strong></header>
    </article>
    <article class="metric-card compact">
      <header><span>Mode</span><strong>{modeValue && modeValue !== '-' ? modeValue : '--'}</strong></header>
    </article>
    <article class="metric-card compact">
      <header><span>Type</span><strong>{typeValue && typeValue !== '-' ? typeValue : '--'}</strong></header>
    </article>
  </section>

  <section class="control-grid">
    {#if outputWritable && outputRow}
      <article class="control-card">
        <h5>Output State</h5>
        <div class="control-row compact">
          <button
            type="button"
            class="toggle-pill"
            aria-pressed={outputDraft}
            onclick={() => {
              outputDraft = !outputDraft;
            }}
          >
            <span class="track"><span class="thumb"></span></span>
            <span class="label">{outputDraft ? 'On' : 'Off'}</span>
          </button>
          <button class="btn btn-primary" onclick={sendOutput}>Set</button>
        </div>
      </article>
    {/if}

    {#if invertedWritable && invertedRow}
      <article class="control-card">
        <h5>Inverted</h5>
        <div class="control-row compact">
          <button
            type="button"
            class="toggle-pill"
            aria-pressed={invertedDraft}
            onclick={() => {
              invertedDraft = !invertedDraft;
            }}
          >
            <span class="track"><span class="thumb"></span></span>
            <span class="label">{invertedDraft ? 'Inverted' : 'Normal'}</span>
          </button>
          <button class="btn btn-primary" onclick={sendInverted}>Set</button>
        </div>
      </article>
    {/if}

    {#if channelWritable && channelRow}
      <article class="control-card">
        <h5>Channel</h5>
        <div class="control-row compact">
          <input
            type="number"
            step="1"
            value={channelDraft}
            oninput={(event) => {
              channelDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendChannel}>Set</button>
        </div>
      </article>
    {/if}

    {#if portWritable && portRow}
      <article class="control-card">
        <h5>Port</h5>
        <div class="control-row compact">
          <input
            type="number"
            step="1"
            value={portDraft}
            oninput={(event) => {
              portDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendPort}>Set</button>
        </div>
      </article>
    {/if}

    {#if modeWritable && modeRow}
      <article class="control-card">
        <h5>Mode</h5>
        <div class="control-row compact">
          <input
            value={modeDraft}
            placeholder={modeRow.signal_type === 'bool' ? 'true/false' : 'input/output'}
            oninput={(event) => {
              modeDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendMode}>Set</button>
        </div>
      </article>
    {/if}

    {#if !outputWritable && !invertedWritable && !channelWritable && !portWritable && !modeWritable}
      <article class="control-card empty">
        <h5>Writable Controls</h5>
        <p>No writable DIO controls were discovered for this signal group.</p>
      </article>
    {/if}
  </section>

  {#if showExtraFieldsSection}
    <section class="extra-section">
      <header>
        <h5>Other DIO Fields</h5>
        <span>{extraSignals.length}</span>
      </header>
      <div class="extra-list">
        {#each extraSignals as row (row.signal_id)}
          {@const kind = extraInputKind(row)}
          <article class="extra-row">
            <div class="extra-meta">
              <strong>{leafPath(row.path)}</strong>
              <span>{row.signal_type}</span>
              <span>{row.access}</span>
            </div>
            <code>{formatValue(row.value)}</code>
            {#if row.access === 'write'}
              <div class="extra-controls">
                {#if kind === 'bool'}
                  <select
                    value={extraDraftBySignal[row.signal_id] ?? (boolFor(row.signal_id) ? 'true' : 'false')}
                    onchange={(event) => setExtraDraft(row.signal_id, (event.currentTarget as HTMLSelectElement).value)}
                  >
                    <option value="true">true</option>
                    <option value="false">false</option>
                  </select>
                {:else}
                  <input
                    value={extraDraftBySignal[row.signal_id] ?? (row.value === '-' ? '' : row.value)}
                    oninput={(event) => setExtraDraft(row.signal_id, (event.currentTarget as HTMLInputElement).value)}
                  />
                {/if}
                <button class="btn btn-mini btn-primary" onclick={() => applyExtra(row)}>Set</button>
              </div>
            {/if}
          </article>
        {/each}
      </div>
    </section>
  {/if}
</div>

<style>
  .dio-root {
    min-height: 100%;
    display: grid;
    grid-template-rows: auto auto auto minmax(0, 1fr);
    gap: 0.34rem;
  }

  .dio-root.no-extra-fields {
    grid-template-rows: auto auto auto;
  }

  .identity {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: linear-gradient(180deg, rgba(43, 52, 69, 0.6), rgba(27, 33, 44, 0.7));
    padding: 0.4rem;
  }

  .chip-row {
    display: flex;
    flex-wrap: wrap;
    gap: 0.22rem;
    align-items: center;
    margin-bottom: 0.34rem;
  }

  .chip {
    border: 1px solid var(--border-subtle);
    background: var(--surface-3);
    color: var(--text-soft);
    border-radius: 999px;
    padding: 0.08rem 0.38rem;
    font-size: 0.62rem;
    text-transform: uppercase;
    letter-spacing: 0.04em;
  }

  .chip.ok {
    border-color: rgba(34, 197, 94, 0.42);
    background: rgba(20, 83, 45, 0.35);
    color: #bbf7d0;
  }

  .chip.bad {
    border-color: rgba(248, 113, 113, 0.45);
    color: #fecaca;
    background: rgba(127, 29, 29, 0.35);
  }

  .identity-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.3rem 0.42rem;
  }

  .identity-grid div {
    min-width: 0;
    display: grid;
    gap: 0.08rem;
  }

  .identity-grid span {
    font-size: 0.6rem;
    letter-spacing: 0.02em;
    text-transform: uppercase;
    color: var(--text-soft);
  }

  .identity-grid strong {
    min-width: 0;
    font-size: 0.68rem;
    color: var(--text-strong);
    font-family: var(--font-mono);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .identity-grid strong.path {
    white-space: normal;
    overflow-wrap: anywhere;
    line-height: 1.2;
  }

  .metric-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.28rem;
  }

  .metric-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(34, 41, 56, 0.8);
    padding: 0.34rem;
    display: grid;
    gap: 0.26rem;
    align-content: start;
  }

  .metric-card header {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    gap: 0.32rem;
  }

  .metric-card span {
    color: var(--text-soft);
    font-size: 0.62rem;
    letter-spacing: 0.03em;
    text-transform: uppercase;
  }

  .metric-card strong {
    color: var(--text-strong);
    font-family: var(--font-mono);
    font-size: 0.72rem;
  }

  .metric-card.compact {
    background: rgba(27, 33, 44, 0.78);
    padding-bottom: 0.28rem;
  }

  .bar {
    height: 5px;
    border-radius: 999px;
    background: rgba(51, 65, 85, 0.55);
    overflow: hidden;
  }

  .bar span {
    display: block;
    height: 100%;
    background: linear-gradient(90deg, #ef4444, #f87171);
    transition: width 90ms linear;
  }

  .control-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.28rem;
  }

  .control-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(34, 41, 56, 0.72);
    padding: 0.36rem;
    display: flex;
    flex-direction: column;
    gap: 0.3rem;
  }

  .control-card h5 {
    margin: 0;
    font-size: 0.67rem;
    font-family: var(--font-display);
    color: var(--text-strong);
    letter-spacing: 0.02em;
  }

  .control-card.empty p {
    margin: 0;
    font-size: 0.66rem;
    color: var(--text-soft);
  }

  .control-row {
    display: grid;
    grid-template-columns: minmax(0, 1fr) minmax(84px, 0.45fr) auto;
    gap: 0.3rem;
    align-items: center;
  }

  .control-row.compact {
    grid-template-columns: minmax(0, 1fr) auto;
  }

  .control-row input,
  .control-row select {
    width: 100%;
    min-width: 0;
    font-size: 0.68rem;
    padding: 0.28rem 0.36rem;
  }

  .toggle-pill {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    background: rgba(27, 33, 44, 0.82);
    color: var(--text);
    min-height: 1.8rem;
    width: 100%;
    padding: 0.16rem 0.34rem;
    display: inline-flex;
    align-items: center;
    gap: 0.34rem;
    cursor: pointer;
  }

  .toggle-pill:hover {
    border-color: var(--border-emphasis);
  }

  .toggle-pill .track {
    width: 1.8rem;
    height: 1rem;
    border-radius: 999px;
    background: rgba(71, 85, 105, 0.8);
    position: relative;
    flex-shrink: 0;
  }

  .toggle-pill .thumb {
    position: absolute;
    top: 0.1rem;
    left: 0.1rem;
    width: 0.8rem;
    height: 0.8rem;
    border-radius: 50%;
    background: #e2e8f0;
    transition: transform 120ms ease;
  }

  .toggle-pill[aria-pressed='true'] .track {
    background: rgba(220, 38, 38, 0.72);
  }

  .toggle-pill[aria-pressed='true'] .thumb {
    transform: translateX(0.8rem);
    background: #fff1f2;
  }

  .toggle-pill .label {
    font-size: 0.66rem;
    color: var(--text-soft);
  }

  .extra-section {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(19, 24, 35, 0.7);
    min-height: 0;
    display: grid;
    grid-template-rows: auto auto;
  }

  .extra-section header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 0.32rem 0.4rem;
    border-bottom: 1px solid var(--border-subtle);
  }

  .extra-section h5 {
    margin: 0;
    font-size: 0.66rem;
    color: var(--text-strong);
  }

  .extra-section header span {
    font-size: 0.62rem;
    color: var(--text-soft);
    font-family: var(--font-mono);
  }

  .extra-list {
    overflow: visible;
    display: grid;
    gap: 0.24rem;
    padding: 0.34rem;
    align-content: start;
  }

  .extra-row {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: rgba(27, 33, 44, 0.78);
    padding: 0.28rem 0.32rem;
    display: grid;
    gap: 0.22rem;
  }

  .extra-meta {
    display: flex;
    flex-wrap: wrap;
    gap: 0.24rem;
    align-items: center;
  }

  .extra-meta strong {
    font-size: 0.66rem;
    color: var(--text-strong);
  }

  .extra-meta span {
    font-size: 0.6rem;
    color: var(--text-soft);
    text-transform: uppercase;
  }

  .extra-row code {
    font-size: 0.64rem;
    color: var(--text);
    font-family: var(--font-mono);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .extra-controls {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.24rem;
    align-items: center;
  }

  .extra-controls input,
  .extra-controls select {
    width: 100%;
    min-width: 0;
    font-size: 0.64rem;
    padding: 0.22rem 0.3rem;
  }

  .btn.btn-mini {
    font-size: 0.6rem;
    padding: 0.2rem 0.34rem;
  }

  @media (max-width: 780px) {
    .metric-grid,
    .control-grid {
      grid-template-columns: 1fr;
    }

    .identity-grid {
      grid-template-columns: 1fr;
    }
  }
</style>
