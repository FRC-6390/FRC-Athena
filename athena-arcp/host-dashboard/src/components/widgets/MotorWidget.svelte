<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { readMotorConfig } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let commandDraft = $state('0');
  let commandTargetSignalId = $state<number | null>(null);
  let currentLimitDraft = $state('0');
  let canIdDraft = $state('0');
  let canbusDraft = $state('');
  let invertedDraft = $state(false);
  let neutralModeDraft = $state<'Brake' | 'Coast'>('Coast');
  let extraDraftBySignal = $state<Record<number, string>>({});
  let commandSeedSignalId = $state<number | null>(null);
  let commandTargetSeedSignalId = $state<number | null>(null);
  let currentLimitSeedSignalId = $state<number | null>(null);
  let canIdSeedSignalId = $state<number | null>(null);
  let canbusSeedSignalId = $state<number | null>(null);
  let invertedSeedSignalId = $state<number | null>(null);
  let neutralModeSeedSignalId = $state<number | null>(null);

  const config = $derived(readMotorConfig(configRaw, signal, signals));

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

  function clamp01(value: number): number {
    return Math.max(0, Math.min(1, value));
  }

  function isWritable(row: SignalRow | null): row is SignalRow {
    return !!row && row.access === 'write';
  }

  function isNumericSignal(row: SignalRow | null): row is SignalRow {
    return !!row && (row.signal_type === 'f64' || row.signal_type === 'i64');
  }

  function outputModeLabel(row: SignalRow): string {
    const token = normalizeToken(leafPath(row.path));
    if (token.includes('voltage') || token.includes('volt')) return 'Voltage';
    if (token.includes('rpm')) return 'RPM';
    if (token.includes('velocity') || token.includes('speed')) return 'Velocity';
    if (token.includes('percent') || token.includes('duty') || token.includes('output')) return 'Percent';
    if (token.includes('position') || token.includes('rot')) return 'Position';
    if (token.includes('current')) return 'Current';
    return leafPath(row.path);
  }

  function isOutputCommandCandidate(row: SignalRow): boolean {
    if (!isNumericSignal(row) || row.access !== 'write') return false;
    const token = normalizeToken(leafPath(row.path));
    if (!token) return false;
    if (token.includes('currentlimit') || token.includes('limitamps')) return false;
    if (token.includes('canid')) return false;
    return (
      token.includes('command') ||
      token.includes('setpoint') ||
      token.includes('target') ||
      token.includes('output') ||
      token.includes('percent') ||
      token.includes('duty') ||
      token.includes('voltage') ||
      token.includes('volt') ||
      token.includes('velocity') ||
      token.includes('speed') ||
      token.includes('rpm')
    );
  }

  function isGenericOutputCommand(row: SignalRow): boolean {
    const token = normalizeToken(leafPath(row.path));
    return token === 'command' || token === 'set' || token === 'outputcommand' || token === 'outputset';
  }

  function isSpecificOutputCommand(row: SignalRow): boolean {
    const token = normalizeToken(leafPath(row.path));
    return (
      token.includes('percent') ||
      token.includes('duty') ||
      token.includes('voltage') ||
      token.includes('volt') ||
      token.includes('velocity') ||
      token.includes('speed') ||
      token.includes('rpm')
    );
  }

  function parseFloatDraft(raw: string): number | null {
    const numeric = Number(raw.trim());
    return Number.isFinite(numeric) ? numeric : null;
  }

  function formatNumber(value: number | null, digits = 2): string {
    if (value === null) return '--';
    return value.toFixed(digits);
  }

  function usagePercent(value: number | null, min: number, max: number): number {
    if (value === null) return 0;
    if (max <= min) return 0;
    return clamp01((value - min) / (max - min));
  }

  function formatValue(raw: string): string {
    if (raw === '-') return '--';
    return raw;
  }

  function sendRaw(row: SignalRow, valueRaw: string) {
    onSendSet(row.signal_id, valueRaw);
  }

  const motorParentPath = $derived(parentPath(signal.path));

  const siblingSignals = $derived.by(() =>
    signals
      .filter((entry) => parentPath(entry.path) === motorParentPath)
      .sort((a, b) => a.path.localeCompare(b.path))
  );

  const canIdRow = $derived(rowFor(config.canIdSignalId));
  const canbusRow = $derived(rowFor(config.canbusSignalId));
  const typeRow = $derived(rowFor(config.typeSignalId));
  const connectedRow = $derived(rowFor(config.connectedSignalId));
  const stalledRow = $derived(rowFor(config.stalledSignalId));
  const neutralModeRow = $derived(rowFor(config.neutralModeSignalId));
  const currentLimitRow = $derived(rowFor(config.currentLimitSignalId));
  const invertedRow = $derived(rowFor(config.invertedSignalId));
  const brakeModeRow = $derived(rowFor(config.brakeModeSignalId));
  const outputRow = $derived(rowFor(config.outputSignalId));
  const velocityRow = $derived(rowFor(config.velocitySignalId));
  const positionRow = $derived(rowFor(config.positionSignalId));
  const currentRow = $derived(rowFor(config.currentSignalId));
  const temperatureRow = $derived(rowFor(config.temperatureSignalId));
  const voltageRow = $derived(rowFor(config.voltageSignalId));
  const explicitCommandRow = $derived(rowFor(config.commandSignalId));

  const commandTargets = $derived.by(() => {
    const rows: SignalRow[] = [];
    const seen = new Set<number>();
    const pushIfCandidate = (row: SignalRow | null) => {
      if (!row) return;
      if (!isOutputCommandCandidate(row)) return;
      if (seen.has(row.signal_id)) return;
      seen.add(row.signal_id);
      rows.push(row);
    };

    pushIfCandidate(explicitCommandRow);
    pushIfCandidate(outputRow);
    for (const row of siblingSignals) {
      pushIfCandidate(row);
    }
    const hasSpecific = rows.some((row) => isSpecificOutputCommand(row));
    if (hasSpecific) {
      return rows.filter((row) => !isGenericOutputCommand(row));
    }
    return rows;
  });

  const commandSignal = $derived.by(() => {
    if (commandTargetSignalId !== null) {
      const selected = commandTargets.find((row) => row.signal_id === commandTargetSignalId);
      if (selected) return selected;
    }
    return commandTargets[0] ?? null;
  });

  const commandTargetIds = $derived.by(() => {
    const ids = new Set<number>();
    for (const row of commandTargets) {
      ids.add(row.signal_id);
    }
    return ids;
  });

  const knownSignalIds = $derived.by(() => {
    const ids = [
      config.canIdSignalId,
      config.canbusSignalId,
      config.typeSignalId,
      config.connectedSignalId,
      config.stalledSignalId,
      config.neutralModeSignalId,
      config.currentLimitSignalId,
      config.invertedSignalId,
      config.brakeModeSignalId,
      config.outputSignalId,
      config.velocitySignalId,
      config.positionSignalId,
      config.currentSignalId,
      config.temperatureSignalId,
      config.voltageSignalId,
      config.commandSignalId
    ];
    const next = new Set<number>();
    for (const id of ids) {
      if (typeof id === 'number' && id > 0) next.add(id);
    }
    return next;
  });

  const extraSignals = $derived.by(() =>
    siblingSignals.filter(
      (entry) => !knownSignalIds.has(entry.signal_id) && !commandTargetIds.has(entry.signal_id)
    )
  );
  const showExtraFieldsSection = $derived(config.showOtherFields && extraSignals.length > 0);

  const canId = $derived(numberFor(config.canIdSignalId));
  const canbus = $derived(textFor(config.canbusSignalId));
  const typeText = $derived(textFor(config.typeSignalId));
  const connected = $derived(boolFor(config.connectedSignalId));
  const stalled = $derived(boolFor(config.stalledSignalId));
  const neutralModeText = $derived.by(() => {
    const direct = textFor(config.neutralModeSignalId);
    if (direct && direct !== '-') return direct;
    const brake = boolFor(config.brakeModeSignalId);
    if (brake === null) return null;
    return brake ? 'Brake' : 'Coast';
  });

  const output = $derived(numberFor(config.outputSignalId));
  const velocity = $derived(numberFor(config.velocitySignalId));
  const position = $derived(numberFor(config.positionSignalId));
  const current = $derived(numberFor(config.currentSignalId));
  const temperature = $derived(numberFor(config.temperatureSignalId));
  const voltage = $derived(numberFor(config.voltageSignalId));
  const canIdWritable = $derived(isWritable(canIdRow));
  const canbusWritable = $derived(isWritable(canbusRow));
  const commandWritable = $derived(isWritable(commandSignal));
  const currentLimitValue = $derived(numberFor(config.currentLimitSignalId));
  const currentLimitWritable = $derived(isWritable(currentLimitRow));
  const invertedValue = $derived(boolFor(config.invertedSignalId));
  const invertedWritable = $derived(isWritable(invertedRow));
  const neutralModeWritable = $derived(isWritable(neutralModeRow));
  const neutralModeFallbackWritable = $derived(isWritable(brakeModeRow));

  const neutralModeControl = $derived.by(() => {
    if (neutralModeWritable) return 'neutral_mode';
    if (neutralModeFallbackWritable) return 'brake_mode';
    return null;
  });

  const commandRange = $derived.by(() => {
    const row = commandSignal;
    const token = normalizeToken(row ? leafPath(row.path) : '');
    if (token.includes('voltage') || token.includes('volt')) {
      return { min: -12, max: 12, step: 0.1 };
    }
    if (token.includes('velocity') || token.includes('speed') || token.includes('rpm')) {
      return { min: -6000, max: 6000, step: 1 };
    }
    return { min: -1, max: 1, step: 0.01 };
  });

  const outputUsage = $derived(clamp01(((output ?? 0) + 1) / 2));
  const currentUsage = $derived(usagePercent(current, 0, 80));
  const tempUsage = $derived(usagePercent(temperature, 20, 100));
  const voltageUsage = $derived(usagePercent(voltage, 0, 14));

  $effect(() => {
    if (commandTargets.length === 0) {
      commandTargetSeedSignalId = null;
      commandTargetSignalId = null;
      return;
    }
    const selectedStillExists =
      commandTargetSignalId !== null &&
      commandTargets.some((row) => row.signal_id === commandTargetSignalId);
    const nextSignalId = selectedStillExists
      ? commandTargetSignalId
      : commandTargets[0]?.signal_id ?? null;
    if (nextSignalId === null) return;
    if (commandTargetSeedSignalId === nextSignalId) return;
    commandTargetSeedSignalId = nextSignalId;
    commandTargetSignalId = nextSignalId;
  });

  $effect(() => {
    if (!commandWritable || !commandSignal) {
      commandSeedSignalId = null;
      return;
    }
    if (commandSeedSignalId === commandSignal.signal_id) return;
    commandSeedSignalId = commandSignal.signal_id;
    const seed = numberFor(commandSignal.signal_id) ?? output ?? 0;
    commandDraft = seed.toFixed(3);
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
    if (!currentLimitWritable || !currentLimitRow) {
      currentLimitSeedSignalId = null;
      return;
    }
    if (currentLimitSeedSignalId === currentLimitRow.signal_id) return;
    currentLimitSeedSignalId = currentLimitRow.signal_id;
    currentLimitDraft = String(currentLimitValue ?? 0);
  });

  $effect(() => {
    if (!invertedWritable || !invertedRow) {
      invertedSeedSignalId = null;
      return;
    }
    if (invertedSeedSignalId === invertedRow.signal_id) return;
    invertedSeedSignalId = invertedRow.signal_id;
    invertedDraft = invertedValue ?? false;
  });

  $effect(() => {
    if (!neutralModeControl) {
      neutralModeSeedSignalId = null;
      return;
    }
    const seedSignal = neutralModeControl === 'neutral_mode' ? neutralModeRow : brakeModeRow;
    if (!seedSignal) return;
    if (neutralModeSeedSignalId === seedSignal.signal_id) return;
    neutralModeSeedSignalId = seedSignal.signal_id;
    const mode = neutralModeText?.trim().toLowerCase();
    neutralModeDraft = mode === 'brake' ? 'Brake' : 'Coast';
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

  function sendCommand() {
    if (!commandSignal || !commandWritable) return;
    const parsed = parseFloatDraft(commandDraft);
    if (parsed === null) return;
    const clamped = Math.max(commandRange.min, Math.min(commandRange.max, parsed));
    if (commandSignal.signal_type === 'i64') {
      sendRaw(commandSignal, String(Math.round(clamped)));
      return;
    }
    sendRaw(commandSignal, String(clamped));
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

  function sendCurrentLimit() {
    if (!currentLimitRow || !currentLimitWritable) return;
    const parsed = parseFloatDraft(currentLimitDraft);
    if (parsed === null) return;
    sendRaw(currentLimitRow, String(parsed));
  }

  function sendInverted() {
    if (!invertedRow || !invertedWritable) return;
    sendRaw(invertedRow, invertedDraft ? 'true' : 'false');
  }

  function sendNeutralMode() {
    if (neutralModeControl === 'neutral_mode' && neutralModeRow) {
      if (neutralModeRow.signal_type === 'bool') {
        sendRaw(neutralModeRow, neutralModeDraft === 'Brake' ? 'true' : 'false');
      } else {
        sendRaw(neutralModeRow, neutralModeDraft);
      }
      return;
    }
    if (neutralModeControl === 'brake_mode' && brakeModeRow) {
      sendRaw(brakeModeRow, neutralModeDraft === 'Brake' ? 'true' : 'false');
    }
  }

  function extraInputKind(row: SignalRow): 'bool' | 'number' | 'text' {
    if (row.signal_type === 'bool') return 'bool';
    if (row.signal_type === 'f64' || row.signal_type === 'i64') return 'number';
    return 'text';
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
    if (extraInputKind(row) === 'number') {
      const parsed = parseFloatDraft(raw);
      if (parsed === null) return;
      sendRaw(row, String(parsed));
      return;
    }
    if (extraInputKind(row) === 'bool') {
      const normalized = raw.trim().toLowerCase();
      if (normalized !== 'true' && normalized !== 'false') return;
      sendRaw(row, normalized);
      return;
    }
    if (!raw.trim()) return;
    sendRaw(row, raw);
  }
</script>

<div class={`motor-root ${showExtraFieldsSection ? '' : 'no-extra-fields'}`}>
  <section class="identity">
    <div class="chip-row">
      <span class={`chip ${connected === false ? 'bad' : connected === true ? 'ok' : ''}`}>
        {connected === null ? 'unknown' : connected ? 'connected' : 'disconnected'}
      </span>
      {#if stalled !== null}
        <span class={`chip ${stalled ? 'warn' : ''}`}>{stalled ? 'stalled' : 'not stalled'}</span>
      {/if}
      {#if neutralModeText}
        <span class="chip">{neutralModeText}</span>
      {/if}
      {#if typeText && typeText !== '-'}
        <span class="chip">{typeText}</span>
      {/if}
    </div>

    <div class="identity-grid">
      <div><span>CAN ID</span><strong>{formatNumber(canId, 0)}</strong></div>
      <div><span>CAN Bus</span><strong>{canbus && canbus !== '-' ? canbus : '--'}</strong></div>
      <div><span>Leaf</span><strong>{leafPath(signal.path)}</strong></div>
      <div><span>Path</span><strong class="path">{motorParentPath || signal.path}</strong></div>
    </div>
  </section>

  <section class="metric-grid">
    <article class="metric-card">
      <header><span>Output</span><strong>{formatNumber(output, 3)}</strong></header>
      <div class="bar"><span style={`width:${(outputUsage * 100).toFixed(1)}%;`}></span></div>
    </article>
    <article class="metric-card">
      <header><span>Current A</span><strong>{formatNumber(current, 1)}</strong></header>
      <div class="bar"><span style={`width:${(currentUsage * 100).toFixed(1)}%;`}></span></div>
    </article>
    <article class="metric-card">
      <header><span>Temp C</span><strong>{formatNumber(temperature, 1)}</strong></header>
      <div class="bar"><span style={`width:${(tempUsage * 100).toFixed(1)}%;`}></span></div>
    </article>
    <article class="metric-card">
      <header><span>Voltage</span><strong>{formatNumber(voltage, 2)}</strong></header>
      <div class="bar"><span style={`width:${(voltageUsage * 100).toFixed(1)}%;`}></span></div>
    </article>
    <article class="metric-card compact">
      <header><span>Velocity</span><strong>{formatNumber(velocity, 2)}</strong></header>
    </article>
    <article class="metric-card compact">
      <header><span>Position</span><strong>{formatNumber(position, 2)}</strong></header>
    </article>
  </section>

  <section class="control-grid">
    {#if commandWritable && commandSignal}
      <article class="control-card">
        <h5>Output Command</h5>
        {#if commandTargets.length > 1}
          <div class="mode-row">
            <span>Target</span>
            <select
              value={commandSignal.signal_id}
              onchange={(event) => {
                const raw = Number((event.currentTarget as HTMLSelectElement).value);
                commandTargetSignalId = Number.isFinite(raw) ? raw : commandTargetSignalId;
              }}
            >
              {#each commandTargets as row (row.signal_id)}
                <option value={row.signal_id}>{outputModeLabel(row)} - {leafPath(row.path)}</option>
              {/each}
            </select>
          </div>
        {/if}
        <div class="control-row">
          <input
            type="range"
            min={commandRange.min}
            max={commandRange.max}
            step={commandRange.step}
            value={commandDraft}
            oninput={(event) => {
              commandDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <input
            value={commandDraft}
            oninput={(event) => {
              commandDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendCommand}>Set</button>
        </div>
      </article>
    {/if}

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

    {#if currentLimitWritable && currentLimitRow}
      <article class="control-card">
        <h5>Current Limit A</h5>
        <div class="control-row compact">
          <input
            value={currentLimitDraft}
            oninput={(event) => {
              currentLimitDraft = (event.currentTarget as HTMLInputElement).value;
            }}
          />
          <button class="btn btn-primary" onclick={sendCurrentLimit}>Set</button>
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
            <span class="track">
              <span class="thumb"></span>
            </span>
            <span class="label">{invertedDraft ? 'Inverted' : 'Normal'}</span>
          </button>
          <button class="btn btn-primary" onclick={sendInverted}>Set</button>
        </div>
      </article>
    {/if}

    {#if neutralModeControl}
      <article class="control-card">
        <h5>Neutral Mode</h5>
        <div class="control-row compact">
          <select
            value={neutralModeDraft}
            onchange={(event) => {
              neutralModeDraft =
                (event.currentTarget as HTMLSelectElement).value === 'Brake' ? 'Brake' : 'Coast';
            }}
          >
            <option value="Coast">Coast</option>
            <option value="Brake">Brake</option>
          </select>
          <button class="btn btn-primary" onclick={sendNeutralMode}>Set</button>
        </div>
      </article>
    {/if}

    {#if !commandWritable && !canIdWritable && !canbusWritable && !currentLimitWritable && !invertedWritable && !neutralModeControl}
      <article class="control-card empty">
        <h5>Writable Controls</h5>
        <p>No writable motor controls were discovered for this signal group.</p>
      </article>
    {/if}
  </section>

  {#if showExtraFieldsSection}
    <section class="extra-section">
      <header>
        <h5>Other Motor Fields</h5>
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
  .motor-root {
    min-height: 100%;
    display: grid;
    grid-template-rows: auto auto auto minmax(0, 1fr);
    gap: 0.34rem;
  }

  .motor-root.no-extra-fields {
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

  .chip.warn {
    border-color: rgba(248, 113, 113, 0.45);
    color: #fecaca;
    background: rgba(127, 29, 29, 0.35);
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

  .mode-row {
    display: grid;
    gap: 0.2rem;
  }

  .mode-row span {
    font-size: 0.6rem;
    color: var(--text-soft);
    text-transform: uppercase;
    letter-spacing: 0.03em;
  }

  .mode-row select {
    width: 100%;
    min-width: 0;
    font-size: 0.66rem;
    padding: 0.24rem 0.32rem;
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

  .control-row input[type='range'] {
    padding: 0;
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
    grid-template-rows: auto auto;
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
    overflow: visible;
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
