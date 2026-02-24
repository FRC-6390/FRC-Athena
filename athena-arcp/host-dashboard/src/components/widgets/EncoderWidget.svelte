<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readEncoderConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let ratioDraft = $state('1.0');
  let offsetDraft = $state('0.0');
  let canIdDraft = $state('0');
  let canbusDraft = $state('');
  let invertedDraft = $state(false);
  let ratioSeedSignalId = $state<number | null>(null);
  let offsetSeedSignalId = $state<number | null>(null);
  let canIdSeedSignalId = $state<number | null>(null);
  let canbusSeedSignalId = $state<number | null>(null);
  let invertedSeedSignalId = $state<number | null>(null);
  let extraDraftBySignal = $state<Record<number, string>>({});

  const config = $derived(readEncoderConfig(configRaw, signal, signals));

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

  function numberFor(signalId: number | null): number | null {
    const row = rowFor(signalId);
    if (!row) return null;
    const value = Number(row.value);
    return Number.isFinite(value) ? value : null;
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

  function textFor(signalId: number | null): string | null {
    const row = rowFor(signalId);
    if (!row) return null;
    return row.value;
  }

  function parseFloatDraft(raw: string): number | null {
    const parsed = Number(raw.trim());
    return Number.isFinite(parsed) ? parsed : null;
  }

  function isWritable(row: SignalRow | null): row is SignalRow {
    return !!row && row.access === 'write';
  }

  function formatNumber(value: number | null, digits = 3): string {
    if (value === null) return '--';
    return value.toFixed(digits);
  }

  function clamp01(value: number): number {
    return Math.max(0, Math.min(1, value));
  }

  function wrap360(value: number): number {
    const wrapped = ((value % 360) + 360) % 360;
    return Number.isFinite(wrapped) ? wrapped : 0;
  }

  function formatValue(raw: string): string {
    if (raw === '-') return '--';
    return raw;
  }

  function usagePercent(value: number | null, min: number, max: number): number {
    if (value === null) return 0;
    if (max <= min) return 0;
    return clamp01((value - min) / (max - min));
  }

  function sendRaw(row: SignalRow, valueRaw: string) {
    onSendSet(row.signal_id, valueRaw);
  }

  type EncoderViewMode = 'continuous' | 'zero_to_one' | 'neg180_to_180' | 'zero_to_360' | 'distance';
  type DialSpec = {
    angleDeg: number;
    normalized: number;
    valueLabel: string;
    modeLabel: string;
  };

  function formatValueLabel(value: number, unit: string, digits = 3): string {
    const suffix = unit.trim();
    if (!suffix) return value.toFixed(digits);
    return `${value.toFixed(digits)} ${suffix}`;
  }

  function resolveDialSpec(
    value: number | null,
    mode: EncoderViewMode,
    min: number,
    max: number,
    unit: string
  ): DialSpec {
    if (value === null) {
      return {
        angleDeg: 0,
        normalized: 0,
        valueLabel: '--',
        modeLabel: mode
      };
    }

    if (mode === 'continuous') {
      const normalized = ((value % 1) + 1) % 1;
      return {
        angleDeg: normalized * 360,
        normalized,
        valueLabel: formatValueLabel(value, unit || 'rot', 3),
        modeLabel: 'continuous'
      };
    }

    if (mode === 'zero_to_one') {
      const normalized = clamp01(value);
      return {
        angleDeg: normalized * 360,
        normalized,
        valueLabel: formatValueLabel(value, unit || 'rot', 3),
        modeLabel: '0..1'
      };
    }

    if (mode === 'neg180_to_180') {
      const normalized = clamp01((value + 180) / 360);
      return {
        angleDeg: wrap360(value),
        normalized,
        valueLabel: formatValueLabel(value, unit || 'deg', 1),
        modeLabel: '-180..180'
      };
    }

    if (mode === 'zero_to_360') {
      const normalized = clamp01(value / 360);
      return {
        angleDeg: wrap360(value),
        normalized,
        valueLabel: formatValueLabel(value, unit || 'deg', 1),
        modeLabel: '0..360'
      };
    }

    const safeMin = Number.isFinite(min) ? min : 0;
    const safeMax = Number.isFinite(max) && max > safeMin ? max : safeMin + 1;
    const normalized = clamp01((value - safeMin) / (safeMax - safeMin));
    return {
      angleDeg: normalized * 360,
      normalized,
      valueLabel: formatValueLabel(value, unit, 3),
      modeLabel: `${safeMin.toFixed(2)}..${safeMax.toFixed(2)}`
    };
  }

  function findSiblingByTokens(tokens: string[], allowTypes: string[]): SignalRow | null {
    const normalizedTokens = tokens.map((entry) => normalizeToken(entry));
    for (const row of siblingSignals) {
      if (!allowTypes.includes(row.signal_type)) continue;
      const token = normalizeToken(leafPath(row.path));
      if (normalizedTokens.includes(token)) return row;
    }
    for (const row of siblingSignals) {
      if (!allowTypes.includes(row.signal_type)) continue;
      const token = normalizeToken(leafPath(row.path));
      if (normalizedTokens.some((candidate) => token.includes(candidate) || token.endsWith(candidate))) {
        return row;
      }
    }
    return null;
  }

  function extraInputKind(row: SignalRow): 'bool' | 'number' | 'text' {
    if (row.signal_type === 'bool') return 'bool';
    if (row.signal_type === 'f64' || row.signal_type === 'i64') return 'number';
    return 'text';
  }

  const encoderParentPath = $derived(parentPath(signal.path));
  const siblingSignals = $derived.by(() =>
    signals
      .filter((entry) => parentPath(entry.path) === encoderParentPath)
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const positionRow = $derived(rowFor(config.positionSignalId));
  const velocityRow = $derived(rowFor(config.velocitySignalId));
  const absoluteRow = $derived(rowFor(config.absoluteSignalId));
  const connectedRow = $derived(rowFor(config.connectedSignalId));
  const ratioRow = $derived(
    rowFor(config.ratioSignalId) ??
      findSiblingByTokens(['gear_ratio', 'ratio', 'conversion'], ['f64', 'i64'])
  );
  const offsetRow = $derived(
    rowFor(config.offsetSignalId) ??
      findSiblingByTokens(['offset_rot', 'offset', 'zero_offset', 'calibration_offset'], ['f64', 'i64'])
  );

  const canIdRow = $derived(
    rowFor(config.canIdSignalId) ??
      findSiblingByTokens(['can_id', 'canid', 'id'], ['i64', 'f64'])
  );
  const canbusRow = $derived(
    rowFor(config.canbusSignalId) ??
      findSiblingByTokens(['canbus', 'bus'], ['string'])
  );
  const typeRow = $derived(
    rowFor(config.typeSignalId) ??
      findSiblingByTokens(['type', 'encoder_type', 'sensor_type'], ['string'])
  );
  const invertedRow = $derived(
    rowFor(config.invertedSignalId) ??
      findSiblingByTokens(['inverted', 'invert'], ['bool'])
  );
  const supportsSimRow = $derived(
    rowFor(config.supportsSimulationSignalId) ??
      findSiblingByTokens(['supports_simulation', 'sim_supported', 'simulation'], ['bool'])
  );
  const rawAbsoluteRow = $derived(
    rowFor(config.rawAbsoluteSignalId) ??
      findSiblingByTokens(['raw_absolute_rot', 'raw_absolute', 'raw'], ['f64', 'i64'])
  );

  const knownSignalIds = $derived.by(() => {
    const ids = [
      config.positionSignalId,
      config.velocitySignalId,
      config.absoluteSignalId,
      config.connectedSignalId,
      ratioRow?.signal_id ?? null,
      offsetRow?.signal_id ?? null,
      canIdRow?.signal_id ?? null,
      canbusRow?.signal_id ?? null,
      typeRow?.signal_id ?? null,
      invertedRow?.signal_id ?? null,
      supportsSimRow?.signal_id ?? null,
      rawAbsoluteRow?.signal_id ?? null
    ];
    const out = new Set<number>();
    for (const id of ids) {
      if (typeof id === 'number' && id > 0) out.add(id);
    }
    return out;
  });

  const extraSignals = $derived.by(() =>
    siblingSignals.filter((entry) => !knownSignalIds.has(entry.signal_id))
  );
  const showExtraFieldsSection = $derived(extraSignals.length > 0);

  const connected = $derived(boolFor(config.connectedSignalId));
  const position = $derived(numberFor(config.positionSignalId));
  const velocity = $derived(numberFor(config.velocitySignalId));
  const absolute = $derived(numberFor(config.absoluteSignalId));
  const ratio = $derived(ratioRow ? numberFor(ratioRow.signal_id) : null);
  const offset = $derived(offsetRow ? numberFor(offsetRow.signal_id) : null);
  const rawAbsolute = $derived(
    rawAbsoluteRow ? numberFor(rawAbsoluteRow.signal_id) : null
  );

  const canId = $derived(canIdRow ? numberFor(canIdRow.signal_id) : null);
  const canbus = $derived(canbusRow ? textFor(canbusRow.signal_id) : null);
  const typeText = $derived(typeRow ? textFor(typeRow.signal_id) : null);
  const inverted = $derived(invertedRow ? boolFor(invertedRow.signal_id) : null);
  const supportsSimulation = $derived(
    supportsSimRow ? boolFor(supportsSimRow.signal_id) : null
  );

  const ratioWritable = $derived(isWritable(ratioRow));
  const offsetWritable = $derived(isWritable(offsetRow));
  const canIdWritable = $derived(isWritable(canIdRow));
  const canbusWritable = $derived(isWritable(canbusRow));
  const invertedWritable = $derived(isWritable(invertedRow));

  const velocityUsage = $derived(usagePercent(Math.abs(velocity ?? 0), 0, 120));
  const ratioUsage = $derived(usagePercent(ratio, 1, 20));
  const offsetUsage = $derived(clamp01(((offset ?? 0) + 1.0) / 2.0));
  const positionDial = $derived(
    resolveDialSpec(
      position,
      config.positionViewMode,
      config.positionMin,
      config.positionMax,
      config.positionUnit
    )
  );
  const absoluteDial = $derived(
    resolveDialSpec(
      absolute,
      config.absoluteViewMode,
      config.absoluteMin,
      config.absoluteMax,
      config.absoluteUnit
    )
  );

  $effect(() => {
    if (!ratioWritable || !ratioRow) {
      ratioSeedSignalId = null;
      return;
    }
    if (ratioSeedSignalId === ratioRow.signal_id) return;
    ratioSeedSignalId = ratioRow.signal_id;
    ratioDraft = String(ratio ?? 1.0);
  });

  $effect(() => {
    if (!offsetWritable || !offsetRow) {
      offsetSeedSignalId = null;
      return;
    }
    if (offsetSeedSignalId === offsetRow.signal_id) return;
    offsetSeedSignalId = offsetRow.signal_id;
    offsetDraft = String(offset ?? 0.0);
  });

  $effect(() => {
    if (!canIdWritable || !canIdRow) {
      canIdSeedSignalId = null;
      return;
    }
    if (canIdSeedSignalId === canIdRow.signal_id) return;
    canIdSeedSignalId = canIdRow.signal_id;
    canIdDraft = String(Math.round(canId ?? 0));
  });

  $effect(() => {
    if (!canbusWritable || !canbusRow) {
      canbusSeedSignalId = null;
      return;
    }
    if (canbusSeedSignalId === canbusRow.signal_id) return;
    canbusSeedSignalId = canbusRow.signal_id;
    canbusDraft = canbus && canbus !== '-' ? canbus : '';
  });

  $effect(() => {
    if (!invertedWritable || !invertedRow) {
      invertedSeedSignalId = null;
      return;
    }
    if (invertedSeedSignalId === invertedRow.signal_id) return;
    invertedSeedSignalId = invertedRow.signal_id;
    invertedDraft = inverted ?? false;
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

  function sendRatio() {
    if (!ratioRow || !ratioWritable) return;
    const parsed = parseFloatDraft(ratioDraft);
    if (parsed === null) return;
    sendRaw(ratioRow, String(parsed));
  }

  function sendOffset() {
    if (!offsetRow || !offsetWritable) return;
    const parsed = parseFloatDraft(offsetDraft);
    if (parsed === null) return;
    sendRaw(offsetRow, String(parsed));
  }

  function sendCanId() {
    if (!canIdRow || !canIdWritable) return;
    const parsed = Number(canIdDraft.trim());
    if (!Number.isFinite(parsed)) return;
    sendRaw(canIdRow, String(Math.round(parsed)));
  }

  function sendCanbus() {
    if (!canbusRow || !canbusWritable) return;
    const next = canbusDraft.trim();
    if (!next) return;
    sendRaw(canbusRow, next);
  }

  function sendInverted() {
    if (!invertedRow || !invertedWritable) return;
    sendRaw(invertedRow, invertedDraft ? 'true' : 'false');
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
      const parsed = parseFloatDraft(raw);
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

<div class={`encoder-root ${showExtraFieldsSection ? '' : 'no-extra-fields'}`}>
  <section class="identity">
    <div class="chip-row">
      <span class={`chip ${connected === false ? 'bad' : connected === true ? 'ok' : ''}`}>
        {connected === null ? 'unknown' : connected ? 'connected' : 'disconnected'}
      </span>
      {#if typeText && typeText !== '-'}
        <span class="chip">{typeText}</span>
      {/if}
      {#if inverted !== null}
        <span class={`chip ${inverted ? 'warn' : ''}`}>{inverted ? 'inverted' : 'normal'}</span>
      {/if}
      {#if supportsSimulation !== null}
        <span class="chip">{supportsSimulation ? 'sim-ready' : 'no sim'}</span>
      {/if}
    </div>

    <div class="identity-grid">
      <div><span>CAN ID</span><strong>{formatNumber(canId, 0)}</strong></div>
      <div><span>CAN Bus</span><strong>{canbus && canbus !== '-' ? canbus : '--'}</strong></div>
      <div><span>Leaf</span><strong>{leafPath(signal.path)}</strong></div>
      <div><span>Path</span><strong class="path">{encoderParentPath || signal.path}</strong></div>
    </div>
  </section>

  <section class="metric-grid">
    <article class="metric-card dial-card">
      <header><span>Position Pose</span><strong>{positionDial.valueLabel}</strong></header>
      <div class="dial-wrap" aria-label="position pose dial">
        <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
          <circle class="ring" cx="50" cy="50" r="42" />
          <g class="ticks">
            <line x1="50" y1="8" x2="50" y2="13"></line>
            <line x1="50" y1="92" x2="50" y2="87"></line>
            <line x1="8" y1="50" x2="13" y2="50"></line>
            <line x1="92" y1="50" x2="87" y2="50"></line>
          </g>
          <g transform={`rotate(${positionDial.angleDeg.toFixed(2)} 50 50)`}>
            <line class="needle" x1="50" y1="50" x2="50" y2="15" />
            <polygon class="needle-head" points="50,8 46.8,16.5 53.2,16.5" />
          </g>
          <circle class="hub" cx="50" cy="50" r="2.2" />
        </svg>
      </div>
      <div class="bar"><span style={`width:${(positionDial.normalized * 100).toFixed(1)}%;`}></span></div>
      <small>{positionDial.modeLabel}</small>
    </article>
    <article class="metric-card">
      <header><span>Velocity rps</span><strong>{formatNumber(velocity, 3)}</strong></header>
      <div class="bar"><span style={`width:${(velocityUsage * 100).toFixed(1)}%;`}></span></div>
    </article>
    <article class="metric-card dial-card">
      <header><span>Absolute Pose</span><strong>{absoluteDial.valueLabel}</strong></header>
      <div class="dial-wrap" aria-label="absolute pose dial">
        <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
          <circle class="ring" cx="50" cy="50" r="42" />
          <g class="ticks">
            <line x1="50" y1="8" x2="50" y2="13"></line>
            <line x1="50" y1="92" x2="50" y2="87"></line>
            <line x1="8" y1="50" x2="13" y2="50"></line>
            <line x1="92" y1="50" x2="87" y2="50"></line>
          </g>
          <g transform={`rotate(${absoluteDial.angleDeg.toFixed(2)} 50 50)`}>
            <line class="needle" x1="50" y1="50" x2="50" y2="15" />
            <polygon class="needle-head" points="50,8 46.8,16.5 53.2,16.5" />
          </g>
          <circle class="hub" cx="50" cy="50" r="2.2" />
        </svg>
      </div>
      <div class="bar"><span style={`width:${(absoluteDial.normalized * 100).toFixed(1)}%;`}></span></div>
      <small>{absoluteDial.modeLabel}</small>
    </article>
    <article class="metric-card">
      <header><span>Raw abs rot</span><strong>{formatNumber(rawAbsolute, 3)}</strong></header>
    </article>
    <article class="metric-card compact">
      <header><span>Gear ratio</span><strong>{formatNumber(ratio, 4)}</strong></header>
      <div class="bar"><span style={`width:${(ratioUsage * 100).toFixed(1)}%;`}></span></div>
      {#if ratioWritable && ratioRow}
        <div class="inline-edit">
          <input
            value={ratioDraft}
            oninput={(event) => {
              ratioDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-mini btn-primary" onclick={sendRatio}>Set</button>
        </div>
      {/if}
    </article>
    <article class="metric-card compact">
      <header><span>Offset rot</span><strong>{formatNumber(offset, 4)}</strong></header>
      <div class="bar"><span style={`width:${(offsetUsage * 100).toFixed(1)}%;`}></span></div>
      {#if offsetWritable && offsetRow}
        <div class="inline-edit">
          <input
            value={offsetDraft}
            oninput={(event) => {
              offsetDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-mini btn-primary" onclick={sendOffset}>Set</button>
        </div>
      {/if}
    </article>
  </section>

  <section class="control-grid">
    {#if canIdWritable && canIdRow}
      <article class="control-card">
        <h5>CAN ID</h5>
        <div class="control-row compact">
          <input
            type="number"
            step="1"
            value={canIdDraft}
            oninput={(event) => {
              canIdDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendCanId}>Set</button>
        </div>
      </article>
    {/if}

    {#if canbusWritable && canbusRow}
      <article class="control-card">
        <h5>CAN Bus</h5>
        <div class="control-row compact">
          <input
            value={canbusDraft}
            oninput={(event) => {
              canbusDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendCanbus}>Set</button>
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

    {#if !canIdWritable && !canbusWritable && !invertedWritable}
      <article class="control-card empty">
        <h5>Writable Controls</h5>
        <p>No writable encoder controls were discovered for this signal group.</p>
      </article>
    {/if}
  </section>

  {#if showExtraFieldsSection}
    <section class="extra-section">
      <header>
        <h5>Other Encoder Fields</h5>
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
  .encoder-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto auto auto minmax(0, 1fr);
    gap: 0.34rem;
  }

  .encoder-root.no-extra-fields {
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

  .chip.warn,
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

  .metric-card small {
    color: var(--text-soft);
    font-size: 0.6rem;
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  .metric-card.compact {
    background: rgba(27, 33, 44, 0.78);
    padding-bottom: 0.28rem;
  }

  .inline-edit {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.26rem;
    align-items: center;
  }

  .inline-edit input {
    width: 100%;
    min-width: 0;
    font-size: 0.66rem;
    padding: 0.24rem 0.32rem;
  }

  .dial-card {
    align-content: stretch;
  }

  .dial-wrap {
    width: min(128px, 100%);
    margin: 0 auto;
  }

  .dial-wrap svg {
    display: block;
    width: 100%;
    height: auto;
  }

  .dial-wrap .ring {
    fill: rgba(20, 25, 34, 0.92);
    stroke: rgba(100, 116, 139, 0.55);
    stroke-width: 2.1;
  }

  .dial-wrap .ticks line {
    stroke: rgba(148, 163, 184, 0.75);
    stroke-width: 1.8;
    stroke-linecap: round;
  }

  .dial-wrap .needle {
    stroke: #f87171;
    stroke-width: 2.4;
    stroke-linecap: round;
  }

  .dial-wrap .needle-head {
    fill: #ef4444;
  }

  .dial-wrap .hub {
    fill: #fca5a5;
    stroke: rgba(248, 113, 113, 0.65);
    stroke-width: 1.1;
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

  .control-row input {
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
    border: 1px solid rgba(100, 116, 139, 0.55);
    flex: 0 0 auto;
  }

  .toggle-pill .thumb {
    position: absolute;
    top: 1px;
    left: 1px;
    width: 0.74rem;
    height: 0.74rem;
    border-radius: 999px;
    background: #e5e7eb;
    transition: transform 140ms ease;
  }

  .toggle-pill[aria-pressed='true'] .track {
    background: rgba(180, 35, 45, 0.38);
    border-color: rgba(248, 113, 113, 0.55);
  }

  .toggle-pill[aria-pressed='true'] .thumb {
    transform: translateX(0.78rem);
    background: #fee2e2;
  }

  .toggle-pill .label {
    font-size: 0.68rem;
    color: var(--text-strong);
  }

  .extra-section {
    min-height: 0;
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(27, 33, 44, 0.74);
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
  }

  .extra-section > header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    gap: 0.3rem;
    border-bottom: 1px solid var(--border-subtle);
    padding: 0.34rem 0.42rem;
  }

  .extra-section > header h5 {
    margin: 0;
    font-size: 0.67rem;
    font-family: var(--font-display);
    color: var(--text-strong);
  }

  .extra-section > header span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.1rem 0.38rem;
    font-size: 0.62rem;
    color: var(--text-soft);
    background: rgba(43, 52, 69, 0.7);
  }

  .extra-list {
    min-height: 0;
    overflow: auto;
    display: grid;
    gap: 0.3rem;
    padding: 0.4rem;
    align-content: start;
  }

  .extra-row {
    border: 1px solid var(--border-subtle);
    border-radius: 7px;
    background: rgba(34, 41, 56, 0.72);
    padding: 0.34rem;
    display: grid;
    gap: 0.26rem;
  }

  .extra-meta {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 0.2rem;
  }

  .extra-meta strong {
    font-size: 0.67rem;
    color: var(--text-strong);
  }

  .extra-meta span {
    border: 1px solid var(--border-subtle);
    border-radius: 999px;
    padding: 0.08rem 0.32rem;
    font-size: 0.58rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.03em;
    background: rgba(27, 33, 44, 0.8);
  }

  .extra-row code {
    margin: 0;
    font-family: var(--font-mono);
    font-size: 0.67rem;
    color: var(--text);
    white-space: normal;
    overflow-wrap: anywhere;
  }

  .extra-controls {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 0.28rem;
    align-items: center;
  }

  .extra-controls input,
  .extra-controls select {
    width: 100%;
    min-width: 0;
    padding: 0.28rem 0.36rem;
    font-size: 0.66rem;
  }

  @media (max-width: 620px) {
    .metric-grid,
    .control-grid {
      grid-template-columns: minmax(0, 1fr);
    }
  }
</style>
