<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import type { WidgetConfigRecord } from '../../lib/dashboard';
  import { parseNumericArray, readImuConfig, type ImuViewMode } from '../../lib/widget-config';

  type Props = {
    signal: SignalRow;
    signals: SignalRow[];
    signalById: Map<number, SignalRow>;
    configRaw?: WidgetConfigRecord;
    onSendSet: (signalId: number, valueRaw: string) => void;
  };

  type Triplet = [number | null, number | null, number | null];
  type DisplayMode = 'none' | '1d' | '2d' | '3d';

  type VectorCard = {
    key: 'accel' | 'gyro' | 'mag';
    label: string;
    values: Triplet;
    mode: DisplayMode;
    unit: string;
    magnitude: number | null;
    scale: number;
    view3d: {
      origin: { x: number; y: number };
      axisX: { x: number; y: number };
      axisY: { x: number; y: number };
      axisZ: { x: number; y: number };
      end: { x: number; y: number; hasData: boolean };
    };
    view2d: { x: number; y: number; hasData: boolean };
  };

  let { signal, signals, signalById, configRaw, onSendSet }: Props = $props();

  let canIdDraft = $state('0');
  let canbusDraft = $state('');
  let invertedDraft = $state(false);

  let canIdSeedSignalId = $state<number | null>(null);
  let canbusSeedSignalId = $state<number | null>(null);
  let invertedSeedSignalId = $state<number | null>(null);

  let extraDraftBySignal = $state<Record<number, string>>({});

  const config = $derived(readImuConfig(configRaw, signal, signals));

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

  function numberForRow(row: SignalRow | null): number | null {
    if (!row) return null;
    const value = Number(row.value);
    return Number.isFinite(value) ? value : null;
  }

  function numberFor(signalId: number | null): number | null {
    return numberForRow(rowFor(signalId));
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

  function boolForRow(row: SignalRow | null): boolean | null {
    if (!row) return null;
    const normalized = row.value.trim().toLowerCase();
    if (normalized === 'true' || normalized === '1' || normalized === 'yes' || normalized === 'on') return true;
    if (normalized === 'false' || normalized === '0' || normalized === 'no' || normalized === 'off') return false;
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

  function radToDeg(value: number): number {
    return (value * 180) / Math.PI;
  }

  function wrap360(value: number): number {
    const wrapped = ((value % 360) + 360) % 360;
    return Number.isFinite(wrapped) ? wrapped : 0;
  }

  function wrapSigned180(value: number): number {
    const wrapped = ((value + 180) % 360 + 360) % 360 - 180;
    return Number.isFinite(wrapped) ? wrapped : 0;
  }

  function formatNumber(value: number | null, digits = 2): string {
    if (value === null) return '--';
    return value.toFixed(digits);
  }

  function formatValue(raw: string): string {
    if (raw === '-') return '--';
    return raw;
  }

  function sendRaw(row: SignalRow, valueRaw: string) {
    onSendSet(row.signal_id, valueRaw);
  }

  const imuParentPath = $derived(parentPath(signal.path));

  const siblingSignals = $derived.by(() =>
    signals
      .filter((entry) => parentPath(entry.path) === imuParentPath)
      .sort((a, b) => a.path.localeCompare(b.path))
  );

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

  function vectorFromSources(arrayRow: SignalRow | null, xRow: SignalRow | null, yRow: SignalRow | null, zRow: SignalRow | null): Triplet {
    const array = arrayRow ? parseNumericArray(arrayRow.value) : [];
    const x = array[0] ?? numberForRow(xRow);
    const y = array[1] ?? numberForRow(yRow);
    const z = array[2] ?? numberForRow(zRow);
    return [x ?? null, y ?? null, z ?? null];
  }

  function resolveVectorMode(mode: ImuViewMode, values: Triplet): DisplayMode {
    const hasX = values[0] !== null;
    const hasY = values[1] !== null;
    const hasZ = values[2] !== null;
    const hasAny = hasX || hasY || hasZ;
    if (!hasAny) return 'none';

    if (mode === '3d') return '3d';
    if (mode === '2d') return hasX || hasY ? '2d' : '1d';
    if (mode === '1d') return '1d';

    if (hasX && hasY && hasZ) return '3d';
    if (hasX || hasY) return '2d';
    return '1d';
  }

  function resolveOrientationMode(mode: ImuViewMode, roll: number | null, pitch: number | null, yaw: number | null, hasHeading: boolean): DisplayMode {
    const hasAny = roll !== null || pitch !== null || yaw !== null || hasHeading;
    if (!hasAny) return 'none';

    if (mode === '3d') return '3d';
    if (mode === '2d') return hasHeading || yaw !== null ? '2d' : '1d';
    if (mode === '1d') return '1d';

    if (roll !== null && pitch !== null && yaw !== null) return '3d';
    if (hasHeading || yaw !== null) return '2d';
    return '1d';
  }

  function vectorMagnitude(values: Triplet): number | null {
    const [x, y, z] = values;
    if (x === null && y === null && z === null) return null;
    return Math.hypot(x ?? 0, y ?? 0, z ?? 0);
  }

  function vectorScale(values: Triplet): number {
    const [x, y, z] = values;
    const maxAbs = Math.max(Math.abs(x ?? 0), Math.abs(y ?? 0), Math.abs(z ?? 0), 1);
    return maxAbs;
  }

  function project3d(x: number, y: number, z: number): { x: number; y: number } {
    return {
      x: 50 + (x - y) * 20,
      y: 52 + (x + y) * 9 - z * 20
    };
  }

  function vector3dEndpoint(values: Triplet): { x: number; y: number; hasData: boolean } {
    const [x, y, z] = values;
    if (x === null && y === null && z === null) {
      return { x: 50, y: 52, hasData: false };
    }
    const scale = vectorScale(values);
    const px = (x ?? 0) / scale;
    const py = (y ?? 0) / scale;
    const pz = (z ?? 0) / scale;
    const projected = project3d(px, py, pz);
    return { x: projected.x, y: projected.y, hasData: true };
  }

  function vector2dEndpoint(values: Triplet): { x: number; y: number; hasData: boolean } {
    const [x, y] = values;
    if (x === null && y === null) {
      return { x: 50, y: 50, hasData: false };
    }
    const scale = Math.max(Math.abs(x ?? 0), Math.abs(y ?? 0), 1);
    return {
      x: 50 + ((x ?? 0) / scale) * 34,
      y: 50 - ((y ?? 0) / scale) * 34,
      hasData: true
    };
  }

  function axisWidth(value: number | null, scale: number): number {
    if (value === null) return 0;
    const normalized = Math.max(-1, Math.min(1, value / scale));
    return Math.abs(normalized) * 100;
  }

  function orientationRollWidth(value: number | null): number {
    return Math.min(100, Math.abs(wrapSigned180(value ?? 0)) / 1.8);
  }

  function orientationPitchWidth(value: number | null): number {
    return Math.min(100, Math.abs(wrapSigned180(value ?? 0)) / 0.9);
  }

  function orientationYawWidth(value: number | null): number {
    return (wrap360(value ?? 0) / 360) * 100;
  }

  function vector3dModel(values: Triplet) {
    return {
      origin: project3d(0, 0, 0),
      axisX: project3d(1, 0, 0),
      axisY: project3d(0, 1, 0),
      axisZ: project3d(0, 0, 1),
      end: vector3dEndpoint(values)
    };
  }

  const rollRow = $derived(rowFor(config.rollSignalId));
  const pitchRow = $derived(rowFor(config.pitchSignalId));
  const yawRow = $derived(rowFor(config.yawSignalId));
  const headingRow = $derived(rowFor(config.headingSignalId));
  const connectedRow = $derived(rowFor(config.connectedSignalId));

  const accelRow = $derived(
    rowFor(config.accelSignalId) ??
      findSiblingByTokens(['accel_xyz', 'acceleration', 'accel'], ['f64[]', 'i64[]'])
  );
  const gyroRow = $derived(
    rowFor(config.gyroSignalId) ??
      findSiblingByTokens(['gyro_xyz_dps', 'gyro_rates', 'gyro', 'angular_velocity'], ['f64[]', 'i64[]'])
  );
  const magRow = $derived(
    rowFor(config.magSignalId) ??
      findSiblingByTokens(['mag_xyz_ut', 'mag_xyz', 'magnetometer', 'magnetic_field', 'mag'], ['f64[]', 'i64[]'])
  );

  const canIdRow = $derived(
    rowFor(config.canIdSignalId) ??
      findSiblingByTokens(['can_id', 'canid', 'id'], ['f64', 'i64'])
  );
  const canbusRow = $derived(
    rowFor(config.canbusSignalId) ??
      findSiblingByTokens(['canbus', 'bus'], ['string'])
  );
  const typeRow = $derived(
    rowFor(config.typeSignalId) ??
      findSiblingByTokens(['type', 'imu_type', 'sensor_type'], ['string'])
  );
  const invertedRow = $derived(
    rowFor(config.invertedSignalId) ??
      findSiblingByTokens(['inverted', 'invert'], ['bool'])
  );

  const accelXRow = $derived(findSiblingByTokens(['accel_x', 'linear_accel_x', 'acceleration_x'], ['f64', 'i64']));
  const accelYRow = $derived(findSiblingByTokens(['accel_y', 'linear_accel_y', 'acceleration_y'], ['f64', 'i64']));
  const accelZRow = $derived(findSiblingByTokens(['accel_z', 'linear_accel_z', 'acceleration_z'], ['f64', 'i64']));

  const gyroXRow = $derived(findSiblingByTokens(['gyro_x', 'vel_x_dps', 'omega_x'], ['f64', 'i64']));
  const gyroYRow = $derived(findSiblingByTokens(['gyro_y', 'vel_y_dps', 'omega_y'], ['f64', 'i64']));
  const gyroZRow = $derived(findSiblingByTokens(['gyro_z', 'vel_z_dps', 'omega_z'], ['f64', 'i64']));

  const magXRow = $derived(findSiblingByTokens(['mag_x', 'magnetic_x', 'magx'], ['f64', 'i64']));
  const magYRow = $derived(findSiblingByTokens(['mag_y', 'magnetic_y', 'magy'], ['f64', 'i64']));
  const magZRow = $derived(findSiblingByTokens(['mag_z', 'magnetic_z', 'magz'], ['f64', 'i64']));

  const knownSignalIds = $derived.by(() => {
    const ids = [
      config.rollSignalId,
      config.pitchSignalId,
      config.yawSignalId,
      config.headingSignalId,
      config.connectedSignalId,
      config.accelSignalId,
      config.gyroSignalId,
      config.magSignalId,
      config.canIdSignalId,
      config.canbusSignalId,
      config.typeSignalId,
      config.invertedSignalId,
      accelRow?.signal_id ?? null,
      gyroRow?.signal_id ?? null,
      magRow?.signal_id ?? null,
      canIdRow?.signal_id ?? null,
      canbusRow?.signal_id ?? null,
      typeRow?.signal_id ?? null,
      invertedRow?.signal_id ?? null,
      accelXRow?.signal_id ?? null,
      accelYRow?.signal_id ?? null,
      accelZRow?.signal_id ?? null,
      gyroXRow?.signal_id ?? null,
      gyroYRow?.signal_id ?? null,
      gyroZRow?.signal_id ?? null,
      magXRow?.signal_id ?? null,
      magYRow?.signal_id ?? null,
      magZRow?.signal_id ?? null
    ];
    const out = new Set<number>();
    for (const id of ids) {
      if (typeof id === 'number' && id > 0) out.add(id);
    }
    return out;
  });

  const hiddenExtraTokens = new Set(['maxlinearspeed', 'maxradialspeed', 'maxspeedwindowsec']);

  const extraSignals = $derived.by(() =>
    siblingSignals.filter((entry) => {
      if (knownSignalIds.has(entry.signal_id)) return false;
      const token = normalizeToken(leafPath(entry.path));
      for (const hidden of hiddenExtraTokens) {
        if (token.includes(hidden)) return false;
      }
      return true;
    })
  );

  const showExtraFieldsSection = $derived(extraSignals.length > 0);

  const rollRaw = $derived(numberForRow(rollRow));
  const pitchRaw = $derived(numberForRow(pitchRow));
  const yawRaw = $derived(numberForRow(yawRow));
  const headingRaw = $derived(numberForRow(headingRow));

  const roll = $derived(config.units === 'rad' && rollRaw !== null ? radToDeg(rollRaw) : rollRaw);
  const pitch = $derived(config.units === 'rad' && pitchRaw !== null ? radToDeg(pitchRaw) : pitchRaw);
  const yaw = $derived(config.units === 'rad' && yawRaw !== null ? radToDeg(yawRaw) : yawRaw);

  const connected = $derived(boolForRow(connectedRow));
  const heading = $derived(wrap360(config.units === 'rad' && headingRaw !== null ? radToDeg(headingRaw) : (headingRaw ?? yaw ?? 0)));

  const accelVector = $derived(vectorFromSources(accelRow, accelXRow, accelYRow, accelZRow));
  const gyroVector = $derived(vectorFromSources(gyroRow, gyroXRow, gyroYRow, gyroZRow));
  const magVector = $derived(vectorFromSources(magRow, magXRow, magYRow, magZRow));

  const canId = $derived(numberForRow(canIdRow));
  const canbus = $derived(canbusRow ? textFor(canbusRow.signal_id) : null);
  const typeText = $derived(typeRow ? textFor(typeRow.signal_id) : null);
  const inverted = $derived(boolForRow(invertedRow));

  const hasHeading = $derived(headingRaw !== null || yaw !== null);
  const orientationMode = $derived(resolveOrientationMode(config.orientationViewMode, roll, pitch, yaw, hasHeading));

  const vectorCards = $derived.by<VectorCard[]>(() => [
    {
      key: 'accel',
      label: 'Accel',
      values: accelVector,
      mode: resolveVectorMode(config.accelViewMode, accelVector),
      unit: 'm/s^2',
      magnitude: vectorMagnitude(accelVector),
      scale: vectorScale(accelVector),
      view3d: vector3dModel(accelVector),
      view2d: vector2dEndpoint(accelVector)
    },
    {
      key: 'gyro',
      label: 'Gyro',
      values: gyroVector,
      mode: resolveVectorMode(config.gyroViewMode, gyroVector),
      unit: config.units === 'rad' ? 'rad/s' : 'deg/s',
      magnitude: vectorMagnitude(gyroVector),
      scale: vectorScale(gyroVector),
      view3d: vector3dModel(gyroVector),
      view2d: vector2dEndpoint(gyroVector)
    },
    {
      key: 'mag',
      label: 'Mag',
      values: magVector,
      mode: resolveVectorMode(config.magViewMode, magVector),
      unit: 'uT',
      magnitude: vectorMagnitude(magVector),
      scale: vectorScale(magVector),
      view3d: vector3dModel(magVector),
      view2d: vector2dEndpoint(magVector)
    }
  ]);

  const cubeTransform = $derived(
    `transform: rotateZ(${(yaw ?? heading).toFixed(2)}deg) rotateX(${(pitch ?? 0).toFixed(2)}deg) rotateY(${(roll ?? 0).toFixed(2)}deg);`
  );

  const canIdWritable = $derived(isWritable(canIdRow));
  const canbusWritable = $derived(isWritable(canbusRow));
  const invertedWritable = $derived(isWritable(invertedRow));

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

<div class={`imu-root ${showExtraFieldsSection ? '' : 'no-extra-fields'}`}>
  <section class="identity">
    <div class="chip-row">
      <span class={`chip ${connected === false ? 'bad' : connected === true ? 'ok' : ''}`}>
        {connected === null ? 'unknown' : connected ? 'connected' : 'disconnected'}
      </span>
      <span class="chip">{config.units}</span>
      {#if typeText && typeText !== '-'}
        <span class="chip">{typeText}</span>
      {/if}
      {#if inverted !== null}
        <span class={`chip ${inverted ? 'warn' : ''}`}>{inverted ? 'inverted' : 'normal'}</span>
      {/if}
      <span class="chip">view {orientationMode}</span>
    </div>

    <div class="identity-grid">
      <div><span>CAN ID</span><strong>{formatNumber(canId, 0)}</strong></div>
      <div><span>CAN Bus</span><strong>{canbus && canbus !== '-' ? canbus : '--'}</strong></div>
      <div><span>Leaf</span><strong>{leafPath(signal.path)}</strong></div>
      <div><span>Path</span><strong class="path">{imuParentPath || signal.path}</strong></div>
    </div>
  </section>

  <section class="metric-grid">
    <article class="metric-card orientation-card">
      <header>
        <span>Orientation</span>
        <strong>{heading.toFixed(1)} deg</strong>
      </header>

      {#if orientationMode === '3d'}
        <div class="orientation-3d" aria-label="3d imu orientation">
          <div class="scene">
            <div class="cube" style={cubeTransform}>
              <div class="face front">F</div>
              <div class="face back">B</div>
              <div class="face right">R</div>
              <div class="face left">L</div>
              <div class="face top">U</div>
              <div class="face bottom">D</div>
            </div>
          </div>
        </div>
      {:else if orientationMode === '2d'}
        <div class="orientation-2d" aria-label="heading compass">
          <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
            <circle class="ring" cx="50" cy="50" r="42" />
            <text class="cardinal" x="50" y="12">N</text>
            <text class="cardinal" x="87" y="53">E</text>
            <text class="cardinal" x="50" y="92">S</text>
            <text class="cardinal" x="13" y="53">W</text>
            <g transform={`rotate(${heading.toFixed(2)} 50 50)`}>
              <line class="needle" x1="50" y1="50" x2="50" y2="15" />
              <polygon class="needle-head" points="50,8 46.8,16.5 53.2,16.5" />
            </g>
            <circle class="hub" cx="50" cy="50" r="2.2" />
          </svg>
        </div>
      {:else if orientationMode === '1d'}
        <div class="orientation-1d">
          <div class="axis-strip">
            <span>Roll</span>
            <strong>{formatNumber(roll, 1)}</strong>
            <div class="bar"><span style={`width:${orientationRollWidth(roll).toFixed(1)}%;`}></span></div>
          </div>
          <div class="axis-strip">
            <span>Pitch</span>
            <strong>{formatNumber(pitch, 1)}</strong>
            <div class="bar"><span style={`width:${orientationPitchWidth(pitch).toFixed(1)}%;`}></span></div>
          </div>
          <div class="axis-strip">
            <span>Yaw</span>
            <strong>{formatNumber(yaw ?? heading, 1)}</strong>
            <div class="bar"><span style={`width:${orientationYawWidth(yaw ?? heading).toFixed(1)}%;`}></span></div>
          </div>
        </div>
      {:else}
        <p class="empty-copy">No orientation signals mapped.</p>
      {/if}

      <footer class="orientation-meta">
        <span>R {formatNumber(roll, 1)} deg</span>
        <span>P {formatNumber(pitch, 1)} deg</span>
        <span>Y {formatNumber(yaw ?? heading, 1)} deg</span>
      </footer>
    </article>

    {#each vectorCards as vector (vector.key)}
      <article class="metric-card vector-card">
        <header>
          <span>{vector.label} vector</span>
          <strong>{formatNumber(vector.magnitude, 2)} {vector.unit}</strong>
        </header>

        {#if vector.mode === '3d'}
          <div class="vector-3d" aria-label={`${vector.label} 3d vector`}>
            <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
              <line class="axis axis-x" x1={vector.view3d.origin.x} y1={vector.view3d.origin.y} x2={vector.view3d.axisX.x} y2={vector.view3d.axisX.y} />
              <line class="axis axis-y" x1={vector.view3d.origin.x} y1={vector.view3d.origin.y} x2={vector.view3d.axisY.x} y2={vector.view3d.axisY.y} />
              <line class="axis axis-z" x1={vector.view3d.origin.x} y1={vector.view3d.origin.y} x2={vector.view3d.axisZ.x} y2={vector.view3d.axisZ.y} />
              <text class="axis-label" x={vector.view3d.axisX.x + 2} y={vector.view3d.axisX.y}>X</text>
              <text class="axis-label" x={vector.view3d.axisY.x - 2} y={vector.view3d.axisY.y}>Y</text>
              <text class="axis-label" x={vector.view3d.axisZ.x} y={vector.view3d.axisZ.y - 2}>Z</text>
              {#if vector.view3d.end.hasData}
                <line class="vector-line" x1={vector.view3d.origin.x} y1={vector.view3d.origin.y} x2={vector.view3d.end.x} y2={vector.view3d.end.y} />
                <circle class="vector-tip" cx={vector.view3d.end.x} cy={vector.view3d.end.y} r="2.4" />
              {/if}
            </svg>
          </div>
        {:else if vector.mode === '2d'}
          <div class="vector-2d" aria-label={`${vector.label} 2d vector`}>
            <svg viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet">
              <circle class="ring" cx="50" cy="50" r="40" />
              <line class="axis" x1="10" y1="50" x2="90" y2="50" />
              <line class="axis" x1="50" y1="10" x2="50" y2="90" />
              {#if vector.view2d.hasData}
                <line class="vector-line" x1="50" y1="50" x2={vector.view2d.x} y2={vector.view2d.y} />
                <circle class="vector-tip" cx={vector.view2d.x} cy={vector.view2d.y} r="2.4" />
              {/if}
            </svg>
          </div>
        {:else if vector.mode === '1d'}
          <div class="vector-1d">
            <div class="axis-strip">
              <span>X</span>
              <strong>{formatNumber(vector.values[0], 2)}</strong>
              <div class="bar"><span style={`width:${axisWidth(vector.values[0], vector.scale).toFixed(1)}%;`}></span></div>
            </div>
            <div class="axis-strip">
              <span>Y</span>
              <strong>{formatNumber(vector.values[1], 2)}</strong>
              <div class="bar"><span style={`width:${axisWidth(vector.values[1], vector.scale).toFixed(1)}%;`}></span></div>
            </div>
            <div class="axis-strip">
              <span>Z</span>
              <strong>{formatNumber(vector.values[2], 2)}</strong>
              <div class="bar"><span style={`width:${axisWidth(vector.values[2], vector.scale).toFixed(1)}%;`}></span></div>
            </div>
          </div>
        {:else}
          <p class="empty-copy">No {vector.label.toLowerCase()} vector data.</p>
        {/if}

        <code>{formatNumber(vector.values[0], 2)}, {formatNumber(vector.values[1], 2)}, {formatNumber(vector.values[2], 2)}</code>
      </article>
    {/each}
  </section>

  <section class="control-grid">
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

    {#if canIdWritable && canIdRow}
      <article class="control-card">
        <h5>CAN ID</h5>
        <div class="control-row compact">
          <input
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

    {#if !invertedWritable && !canIdWritable && !canbusWritable}
      <article class="control-card empty">
        <h5>Writable Controls</h5>
        <p>No writable IMU controls were discovered for this signal group.</p>
      </article>
    {/if}
  </section>

  {#if showExtraFieldsSection}
    <section class="extra-section">
      <header>
        <h5>Other IMU Fields</h5>
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
  .imu-root {
    height: 100%;
    min-height: 0;
    display: grid;
    grid-template-rows: auto minmax(0, 1fr) auto minmax(0, 1fr);
    gap: 0.34rem;
  }

  .imu-root.no-extra-fields {
    grid-template-rows: auto minmax(0, 1fr) auto;
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
    min-height: 0;
    overflow: auto;
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 0.28rem;
    align-content: start;
    padding-right: 0.1rem;
  }

  .metric-card {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: rgba(34, 41, 56, 0.8);
    padding: 0.34rem;
    display: grid;
    gap: 0.26rem;
    align-content: start;
    min-height: 0;
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

  .metric-card code {
    font-family: var(--font-mono);
    font-size: 0.67rem;
    color: var(--text);
    background: rgba(15, 23, 42, 0.8);
    border: 1px solid var(--border-subtle);
    border-radius: 6px;
    padding: 0.12rem 0.28rem;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }

  .orientation-card {
    grid-column: span 2;
  }

  .orientation-3d,
  .orientation-2d,
  .vector-3d,
  .vector-2d {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: #101827;
    min-height: 7.4rem;
    overflow: hidden;
    display: grid;
    place-items: center;
  }

  .orientation-2d svg,
  .vector-3d svg,
  .vector-2d svg {
    width: 100%;
    height: 100%;
    display: block;
  }

  .orientation-3d {
    perspective: 620px;
  }

  .scene {
    --cube-size: clamp(2.4rem, 48%, 6.7rem);
    width: 100%;
    height: 100%;
    min-height: 6.8rem;
    display: grid;
    place-items: center;
  }

  .cube {
    width: var(--cube-size);
    aspect-ratio: 1 / 1;
    position: relative;
    transform-style: preserve-3d;
    transition: transform 110ms linear;
  }

  .face {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    border-radius: 3px;
    border: 1px solid rgba(148, 163, 184, 0.46);
    background: rgba(15, 23, 42, 0.78);
    color: rgba(226, 232, 240, 0.88);
    font-size: 0.56rem;
    font-family: var(--font-mono);
    font-weight: 700;
  }

  .front {
    transform: translateZ(calc(var(--cube-size) / 2));
    background: rgba(248, 113, 113, 0.24);
    border-color: rgba(248, 113, 113, 0.55);
  }

  .back {
    transform: rotateY(180deg) translateZ(calc(var(--cube-size) / 2));
  }

  .right {
    transform: rotateY(90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .left {
    transform: rotateY(-90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .top {
    transform: rotateX(90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .bottom {
    transform: rotateX(-90deg) translateZ(calc(var(--cube-size) / 2));
  }

  .orientation-1d,
  .vector-1d {
    display: grid;
    gap: 0.24rem;
  }

  .axis-strip {
    display: grid;
    grid-template-columns: auto auto minmax(0, 1fr);
    align-items: center;
    gap: 0.28rem;
    min-width: 0;
  }

  .axis-strip span {
    font-size: 0.6rem;
    text-transform: uppercase;
    color: var(--text-soft);
  }

  .axis-strip strong {
    font-family: var(--font-mono);
    font-size: 0.66rem;
    color: var(--text-strong);
  }

  .orientation-meta {
    display: flex;
    flex-wrap: wrap;
    gap: 0.32rem;
    border-top: 1px solid var(--border-subtle);
    padding-top: 0.24rem;
  }

  .orientation-meta span {
    font-family: var(--font-mono);
    font-size: 0.64rem;
    color: var(--text-soft);
    text-transform: uppercase;
  }

  .ring {
    fill: rgba(15, 23, 42, 0.68);
    stroke: rgba(148, 163, 184, 0.52);
    stroke-width: 1.4;
  }

  .cardinal {
    fill: rgba(226, 232, 240, 0.92);
    font-family: var(--font-display);
    font-size: 6px;
    font-weight: 700;
    text-anchor: middle;
    dominant-baseline: middle;
  }

  .needle {
    stroke: rgba(248, 113, 113, 0.96);
    stroke-width: 1.8;
    stroke-linecap: round;
  }

  .needle-head {
    fill: rgba(248, 113, 113, 0.96);
  }

  .hub {
    fill: #e2e8f0;
  }

  .axis {
    stroke: rgba(100, 116, 139, 0.78);
    stroke-width: 1.3;
  }

  .axis.axis-x {
    stroke: rgba(248, 113, 113, 0.82);
  }

  .axis.axis-y {
    stroke: rgba(56, 189, 248, 0.82);
  }

  .axis.axis-z {
    stroke: rgba(52, 211, 153, 0.82);
  }

  .axis-label {
    fill: rgba(226, 232, 240, 0.8);
    font-size: 5px;
    font-family: var(--font-mono);
  }

  .vector-line {
    stroke: rgba(248, 113, 113, 0.96);
    stroke-width: 2;
    stroke-linecap: round;
  }

  .vector-tip {
    fill: rgba(248, 113, 113, 0.96);
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

  .empty-copy {
    margin: 0;
    font-size: 0.66rem;
    color: var(--text-soft);
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

  @media (max-width: 700px) {
    .metric-grid,
    .control-grid {
      grid-template-columns: minmax(0, 1fr);
    }

    .orientation-card {
      grid-column: auto;
    }
  }
</style>
