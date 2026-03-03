
<script lang="ts">
  import type { SignalRow } from '../lib/arcp';

  type LoopParamKey = 'kp' | 'ki' | 'kd' | 'ks' | 'kv' | 'ka' | 'kg' | 'ff' | 'izone';
  type LoopParam = { key: LoopParamKey; label: string; signal: SignalRow };
  type ControlLoop = {
    key: string;
    label: string;
    root: string;
    params: LoopParam[];
    setpoint: SignalRow | null;
    measurement: SignalRow | null;
    output: SignalRow | null;
    error: SignalRow | null;
    mode: SignalRow | null;
    status: SignalRow | null;
    startAction: SignalRow | null;
    stopAction: SignalRow | null;
    extra: SignalRow[];
    signals: SignalRow[];
  };

  type Props = {
    signals: SignalRow[];
    historyBySignal: Map<number, number[]>;
    selectedId: number | null;
    onSelectSignal: (signalId: number) => void;
    onSendSet: (signalId: number, valueRaw: string) => void;
    onTriggerAction: (signalId: number) => void;
    onTrackSignalIds?: (signalIds: number[]) => void;
  };

  const PARAM_ORDER: LoopParamKey[] = ['kp', 'ki', 'kd', 'ks', 'kv', 'ka', 'kg', 'ff', 'izone'];
  const PARAM_LABELS: Record<LoopParamKey, string> = {
    kp: 'kP',
    ki: 'kI',
    kd: 'kD',
    ks: 'kS',
    kv: 'kV',
    ka: 'kA',
    kg: 'kG',
    ff: 'FF',
    izone: 'I Zone'
  };
  const PARAM_ALIASES: Record<LoopParamKey, string[]> = {
    kp: ['kp', 'proportional'],
    ki: ['ki', 'integral'],
    kd: ['kd', 'derivative'],
    ks: ['ks', 'static'],
    kv: ['kv', 'velocity'],
    ka: ['ka', 'accel', 'acceleration'],
    kg: ['kg', 'gravity'],
    ff: ['ff', 'kff', 'feedforward'],
    izone: ['izone', 'integralzone', 'iz']
  };

  let {
    signals,
    historyBySignal,
    selectedId,
    onSelectSignal,
    onSendSet,
    onTriggerAction,
    onTrackSignalIds
  }: Props = $props();

  let query = $state('');
  let selectedLoopKey = $state('');
  let toleranceDraft = $state('0.05');
  let drafts = $state<Record<number, string>>({});
  let trackSig = $state('');

  function split(path: string): string[] {
    return path.split('/').map((p) => p.trim()).filter(Boolean);
  }

  function leaf(path: string): string {
    const parts = split(path);
    return parts[parts.length - 1] ?? '';
  }

  function parent(path: string): string {
    const parts = split(path);
    if (parts.length <= 1) return '';
    return parts.slice(0, -1).join('/');
  }

  function token(raw: string): string {
    return raw.toLowerCase().replace(/[^a-z0-9]+/g, '');
  }

  function title(raw: string): string {
    const clean = raw.replace(/[_-]+/g, ' ').trim();
    if (!clean) return 'Loop';
    return clean
      .split(' ')
      .filter(Boolean)
      .map((piece) => piece[0].toUpperCase() + piece.slice(1))
      .join(' ');
  }

  function isNumeric(signal: SignalRow): boolean {
    return signal.signal_type === 'f64' || signal.signal_type === 'i64';
  }

  function num(signal: SignalRow | null | undefined): number | null {
    if (!signal) return null;
    const parsed = Number(signal.value);
    return Number.isFinite(parsed) ? parsed : null;
  }

  function bool(raw: string): boolean {
    const n = raw.trim().toLowerCase();
    return n === 'true' || n === '1' || n === 'yes' || n === 'on';
  }

  function format(value: number | null): string {
    if (value === null) return 'n/a';
    return Math.abs(value) >= 100 ? value.toFixed(2) : value.toFixed(3);
  }

  function rootFor(path: string): string {
    const parentPath = parent(path);
    if (!parentPath) return path;
    const p = token(leaf(parentPath));
    if (p === 'pid' || p === 'controller' || p === 'control' || p === 'loop' || p === 'closedloop') {
      return parent(parentPath) || parentPath;
    }
    return parentPath;
  }

  function looksRelated(signal: SignalRow): boolean {
    if (signal.path.startsWith('Athena/NT4/')) return false;
    const t = token(signal.path);
    if (!t) return false;
    if (t.includes('pid') || t.includes('feedforward') || t.includes('setpoint') || t.includes('target')) return true;
    if (t.includes('measurement') || t.includes('feedback') || t.includes('output') || t.includes('error')) return true;
    if (t.includes('autotune') || t.includes('sysid')) return true;
    for (const aliases of Object.values(PARAM_ALIASES)) {
      if (aliases.some((alias) => t.includes(alias))) return true;
    }
    return signal.access === 'invoke' && (t.includes('start') || t.includes('stop'));
  }

  function paramKey(signal: SignalRow): LoopParamKey | null {
    const t = token(leaf(signal.path));
    for (const key of PARAM_ORDER) {
      const aliases = PARAM_ALIASES[key];
      for (const alias of aliases) {
        if (t === alias || t.endsWith(alias)) return key;
      }
    }
    return null;
  }

  function score(role: 'setpoint' | 'measurement' | 'output' | 'error' | 'mode' | 'status', signal: SignalRow): number {
    const t = token(leaf(signal.path));
    const p = token(signal.path);
    if (role === 'setpoint') {
      let s = 0;
      if (t.includes('setpoint')) s += 12;
      if (t.includes('target') || t.includes('goal') || t.includes('reference')) s += 10;
      if (signal.access === 'write') s += 3;
      if (!isNumeric(signal)) s -= 8;
      if (t.includes('error') || t.includes('output')) s -= 8;
      if (p.includes('autotune')) s -= 4;
      return s;
    }
    if (role === 'measurement') {
      let s = 0;
      if (t.includes('measurement') || t.includes('measured') || t.includes('feedback')) s += 12;
      if (t.includes('actual') || t.includes('position') || t.includes('velocity')) s += 8;
      if (!isNumeric(signal)) s -= 8;
      if (t.includes('setpoint') || t.includes('output')) s -= 6;
      return s;
    }
    if (role === 'output') {
      let s = 0;
      if (t.includes('output') || t.includes('applied')) s += 12;
      if (t.includes('duty') || t.includes('voltage') || t.includes('percent') || t.includes('effort')) s += 8;
      if (!isNumeric(signal)) s -= 8;
      if (t.includes('setpoint') || t.includes('error')) s -= 6;
      return s;
    }
    if (role === 'error') {
      let s = 0;
      if (t.includes('error') || t.endsWith('err')) s += 14;
      if (!isNumeric(signal)) s -= 8;
      return s;
    }
    if (role === 'mode') {
      return t.includes('mode') ? 8 : 0;
    }
    if (t.includes('status') || t.includes('state') || t.includes('result')) return 8;
    return 0;
  }

  function best(role: 'setpoint' | 'measurement' | 'output' | 'error' | 'mode' | 'status', pool: SignalRow[]): SignalRow | null {
    let bestSignal: SignalRow | null = null;
    let bestScore = Number.NEGATIVE_INFINITY;
    for (const signal of pool) {
      const s = score(role, signal);
      if (s > bestScore && s > 0) {
        bestScore = s;
        bestSignal = signal;
      }
    }
    return bestSignal;
  }

  function bestAction(kind: 'start' | 'stop', pool: SignalRow[]): SignalRow | null {
    let selected: SignalRow | null = null;
    let bestScore = Number.NEGATIVE_INFINITY;
    const wanted = kind === 'start' ? ['start', 'run', 'enable'] : ['stop', 'cancel', 'disable'];
    for (const signal of pool) {
      if (signal.access !== 'invoke') continue;
      const t = token(signal.path);
      let s = 0;
      for (const w of wanted) {
        if (t.includes(w)) s += 10;
      }
      if (t.includes('autotune') || t.includes('sysid')) s += 3;
      if (s > bestScore && s > 0) {
        bestScore = s;
        selected = signal;
      }
    }
    return selected;
  }

  const loops = $derived.by(() => {
    const roots = new Set<string>();
    for (const signal of signals) {
      if (!looksRelated(signal)) continue;
      roots.add(rootFor(signal.path));
    }

    const all = signals.filter((signal) => !signal.path.startsWith('Athena/NT4/'));
    const parsed: ControlLoop[] = [];

    for (const root of roots) {
      const rootDepth = split(root).length;
      const scoped = all.filter((signal) => {
        if (!(signal.path === root || signal.path.startsWith(`${root}/`))) return false;
        return split(signal.path).length <= rootDepth + 4;
      });
      if (scoped.length === 0) continue;

      const paramsMap = new Map<LoopParamKey, SignalRow>();
      for (const signal of scoped) {
        if (!isNumeric(signal)) continue;
        const key = paramKey(signal);
        if (!key) continue;
        const current = paramsMap.get(key);
        if (!current || (current.access !== 'write' && signal.access === 'write')) {
          paramsMap.set(key, signal);
        }
      }

      const params = PARAM_ORDER
        .map((key): LoopParam | null => {
          const signal = paramsMap.get(key);
          if (!signal) return null;
          return { key, label: PARAM_LABELS[key], signal };
        })
        .filter((entry): entry is LoopParam => entry !== null);

      const paramIds = new Set<number>(params.map((param) => param.signal.signal_id));
      const rolePool = scoped.filter((signal) => !paramIds.has(signal.signal_id));
      const setpoint = best('setpoint', rolePool);
      const measurement = best('measurement', rolePool);
      const output = best('output', rolePool);
      const error = best('error', rolePool);
      const mode = best('mode', rolePool);
      const status = best('status', rolePool);
      const startAction = bestAction('start', scoped);
      const stopAction = bestAction('stop', scoped);

      const reserved = new Set<number>([
        ...paramIds,
        setpoint?.signal_id ?? -1,
        measurement?.signal_id ?? -1,
        output?.signal_id ?? -1,
        error?.signal_id ?? -1,
        mode?.signal_id ?? -1,
        status?.signal_id ?? -1,
        startAction?.signal_id ?? -1,
        stopAction?.signal_id ?? -1
      ]);

      const extra = scoped.filter((signal) => signal.access === 'write' && !reserved.has(signal.signal_id));
      const strength = params.length + (setpoint ? 1 : 0) + (measurement ? 1 : 0) + (output ? 1 : 0) + (error ? 1 : 0);
      if (strength < 2) continue;

      parsed.push({
        key: root,
        label: title(leaf(root) || root),
        root,
        params,
        setpoint,
        measurement,
        output,
        error,
        mode,
        status,
        startAction,
        stopAction,
        extra,
        signals: scoped
      });
    }

    return parsed.sort((a, b) => a.label.localeCompare(b.label));
  });

  const visibleLoops = $derived.by(() => {
    const q = query.trim().toLowerCase();
    if (!q) return loops;
    return loops.filter((loop) => {
      if (loop.label.toLowerCase().includes(q)) return true;
      if (loop.root.toLowerCase().includes(q)) return true;
      return loop.signals.some((signal) => signal.path.toLowerCase().includes(q));
    });
  });

  const selectedLoop = $derived(visibleLoops.find((loop) => loop.key === selectedLoopKey) ?? null);
  const tolerance = $derived.by(() => {
    const parsed = Number(toleranceDraft);
    return Number.isFinite(parsed) && parsed >= 0 ? parsed : 0.05;
  });

  const setpointValue = $derived(num(selectedLoop?.setpoint));
  const measurementValue = $derived(num(selectedLoop?.measurement));
  const outputValue = $derived(num(selectedLoop?.output));
  const loopErrorValue = $derived.by(() => {
    const fromSignal = num(selectedLoop?.error);
    if (fromSignal !== null) return fromSignal;
    if (setpointValue === null || measurementValue === null) return null;
    return setpointValue - measurementValue;
  });
  const reached = $derived.by(() => (loopErrorValue === null ? null : Math.abs(loopErrorValue) <= tolerance));

  const trackedIds = $derived.by(() => {
    if (!selectedLoop) return [];
    const ids: number[] = [];
    for (const signal of [selectedLoop.setpoint, selectedLoop.measurement, selectedLoop.output, selectedLoop.error]) {
      if (!signal || !isNumeric(signal)) continue;
      ids.push(signal.signal_id);
    }
    return [...new Set(ids)].sort((a, b) => a - b);
  });

  const graphSeries = $derived.by(() => {
    if (!selectedLoop) return [] as Array<{ key: string; label: string; color: string; signal: SignalRow; values: number[]; latest: number | null }>;
    const descriptors = [
      { key: 'setpoint', label: 'Setpoint', color: '#f87171', signal: selectedLoop.setpoint },
      { key: 'measurement', label: 'Measurement', color: '#60a5fa', signal: selectedLoop.measurement },
      { key: 'output', label: 'Output', color: '#fbbf24', signal: selectedLoop.output },
      { key: 'error', label: 'Error', color: '#34d399', signal: selectedLoop.error }
    ];
    return descriptors
      .filter((item): item is { key: string; label: string; color: string; signal: SignalRow } => !!item.signal && isNumeric(item.signal))
      .map((item) => {
        const values = [...(historyBySignal.get(item.signal.signal_id) ?? [])];
        if (values.length === 0) {
          const current = num(item.signal);
          if (current !== null) values.push(current);
        }
        return {
          ...item,
          values,
          latest: values.length > 0 ? values[values.length - 1] ?? null : null
        };
      });
  });

  const graphRange = $derived.by(() => {
    let min = Number.POSITIVE_INFINITY;
    let max = Number.NEGATIVE_INFINITY;
    let maxSamples = 2;
    for (const series of graphSeries) {
      maxSamples = Math.max(maxSamples, series.values.length);
      for (const value of series.values) {
        min = Math.min(min, value);
        max = Math.max(max, value);
      }
    }
    if (!Number.isFinite(min) || !Number.isFinite(max)) {
      min = -1;
      max = 1;
    }
    if (max <= min) max = min + 1;
    return { min, max, maxSamples };
  });

  function xFor(index: number, total: number): number {
    if (total <= 1) return 50;
    return (index / (total - 1)) * 100;
  }

  function yFor(value: number, min: number, max: number): number {
    const range = Math.max(1e-9, max - min);
    return 35 - ((value - min) / range) * 31;
  }

  function linePath(values: number[], maxSamples: number, min: number, max: number): string {
    if (values.length === 0) return 'M 0 18 L 100 18';
    const offset = Math.max(0, maxSamples - values.length);
    return values
      .map((value, index) => {
        const x = xFor(offset + index, maxSamples);
        const y = yFor(value, min, max);
        return `${index === 0 ? 'M' : 'L'} ${x.toFixed(2)} ${y.toFixed(2)}`;
      })
      .join(' ');
  }

  function draftFor(signal: SignalRow): string {
    const existing = drafts[signal.signal_id];
    if (existing !== undefined) return existing;
    return signal.value === '-' ? '' : signal.value;
  }

  function setDraft(signalId: number, value: string) {
    drafts = { ...drafts, [signalId]: value };
  }

  function apply(signal: SignalRow) {
    const raw = draftFor(signal).trim();
    if (!raw) return;
    onSendSet(signal.signal_id, raw);
  }

  function applyAllParams() {
    if (!selectedLoop) return;
    for (const param of selectedLoop.params) {
      if (param.signal.access === 'write') apply(param.signal);
    }
  }

  $effect(() => {
    if (visibleLoops.length === 0) {
      selectedLoopKey = '';
      return;
    }
    if (!visibleLoops.some((loop) => loop.key === selectedLoopKey)) {
      selectedLoopKey = visibleLoops[0].key;
    }
  });

  $effect(() => {
    if (!selectedLoop) return;
    const next = { ...drafts };
    let changed = false;
    const editable = new Set<number>();
    for (const signal of [...selectedLoop.params.map((entry) => entry.signal), ...selectedLoop.extra, selectedLoop.setpoint, selectedLoop.mode]) {
      if (!signal || signal.access !== 'write') continue;
      editable.add(signal.signal_id);
      if (next[signal.signal_id] === undefined) {
        next[signal.signal_id] = signal.value === '-' ? '' : signal.value;
        changed = true;
      }
    }
    for (const signalId of Object.keys(next).map(Number)) {
      if (!editable.has(signalId)) {
        delete next[signalId];
        changed = true;
      }
    }
    if (changed) drafts = next;
  });

  $effect(() => {
    if (!onTrackSignalIds) return;
    const sig = trackedIds.join(',');
    if (sig === trackSig) return;
    trackSig = sig;
    onTrackSignalIds(trackedIds);
  });
</script>

<section class="control-tuner panel">
  <header>
    <h2>Control Tuner</h2>
    <p>Discover PID/FF loops and tune against live setpoint, measurement, output, and error.</p>
  </header>

  <div class="toolbar">
    <input value={query} placeholder="Search loops" oninput={(event) => (query = (event.currentTarget as HTMLInputElement).value)} />
    <span>{visibleLoops.length} loops</span>
  </div>

  <div class="layout">
    <aside class="loop-list panel">
      {#if visibleLoops.length === 0}
        <p class="empty">No control loops detected.</p>
      {:else}
        {#each visibleLoops as loop (loop.key)}
          <button class={`loop-item ${selectedLoopKey === loop.key ? 'active' : ''}`} onclick={() => (selectedLoopKey = loop.key)}>
            <strong>{loop.label}</strong>
            <small>{loop.params.length} params · {loop.signals.length} signals</small>
            <code>{loop.root}</code>
          </button>
        {/each}
      {/if}
    </aside>

    <section class="detail panel">
      {#if !selectedLoop}
        <p class="empty">Select a loop to tune it.</p>
      {:else}
        <div class="loop-head">
          <h3>{selectedLoop.label}</h3>
          <code>{selectedLoop.root}</code>
        </div>

        <div class="metrics">
          <button class={`metric ${selectedId === selectedLoop.setpoint?.signal_id ? 'selected' : ''}`} onclick={() => selectedLoop.setpoint && onSelectSignal(selectedLoop.setpoint.signal_id)}>
            <span>Setpoint</span><strong>{format(setpointValue)}</strong>
          </button>
          <button class={`metric ${selectedId === selectedLoop.measurement?.signal_id ? 'selected' : ''}`} onclick={() => selectedLoop.measurement && onSelectSignal(selectedLoop.measurement.signal_id)}>
            <span>Measurement</span><strong>{format(measurementValue)}</strong>
          </button>
          <button class={`metric ${selectedId === selectedLoop.output?.signal_id ? 'selected' : ''}`} onclick={() => selectedLoop.output && onSelectSignal(selectedLoop.output.signal_id)}>
            <span>Output</span><strong>{format(outputValue)}</strong>
          </button>
          <button class={`metric ${selectedId === selectedLoop.error?.signal_id ? 'selected' : ''}`} onclick={() => selectedLoop.error && onSelectSignal(selectedLoop.error.signal_id)}>
            <span>Error</span><strong>{format(loopErrorValue)}</strong>
          </button>
        </div>

        <div class="reach" data-state={reached === null ? 'unknown' : reached ? 'ok' : 'off'}>
          <div>
            <strong>Setpoint Reach</strong>
            <p>
              {#if reached === null}
                unavailable
              {:else if reached}
                on target
              {:else}
                off target
              {/if}
            </p>
          </div>
          <label>
            Tol
            <input value={toleranceDraft} oninput={(event) => (toleranceDraft = (event.currentTarget as HTMLInputElement).value)} />
          </label>
        </div>

        <div class="main-grid">
          <article class="graph panel">
            {#if graphSeries.length === 0}
              <p class="empty">No numeric series to graph.</p>
            {:else}
              <svg viewBox="0 0 100 37" preserveAspectRatio="none">
                <line x1="0" y1="4" x2="100" y2="4" class="guide" />
                <line x1="0" y1="19" x2="100" y2="19" class="guide" />
                <line x1="0" y1="34" x2="100" y2="34" class="guide" />
                {#each graphSeries as series (series.key)}
                  <path d={linePath(series.values, graphRange.maxSamples, graphRange.min, graphRange.max)} stroke={series.color} />
                {/each}
              </svg>
              <div class="legend">
                {#each graphSeries as series (series.key)}
                  <button class={`legend-item ${selectedId === series.signal.signal_id ? 'selected' : ''}`} onclick={() => onSelectSignal(series.signal.signal_id)}>
                    <i style={`background:${series.color};`}></i>
                    <span>{series.label}</span>
                    <b>{format(series.latest)}</b>
                  </button>
                {/each}
              </div>
            {/if}
          </article>

          <article class="params panel">
            <header>
              <strong>Tuning</strong>
              <button class="btn" onclick={applyAllParams}>Set all</button>
            </header>
            {#if selectedLoop.params.length === 0}
              <p class="empty">No PID/FF params found.</p>
            {:else}
              {#each selectedLoop.params as param (param.signal.signal_id)}
                <div class={`param-row ${selectedId === param.signal.signal_id ? 'selected' : ''}`}>
                  <button onclick={() => onSelectSignal(param.signal.signal_id)}>{param.label}</button>
                  <input value={draftFor(param.signal)} disabled={param.signal.access !== 'write'} oninput={(event) => setDraft(param.signal.signal_id, (event.currentTarget as HTMLInputElement).value)} />
                  <button class="btn btn-primary" disabled={param.signal.access !== 'write'} onclick={() => apply(param.signal)}>Set</button>
                </div>
              {/each}
            {/if}
          </article>
        </div>

        <div class="extras panel">
          {#if selectedLoop.setpoint && selectedLoop.setpoint.access === 'write'}
            <div class="extra-row">
              <span>Setpoint</span>
              <input value={draftFor(selectedLoop.setpoint)} oninput={(event) => setDraft(selectedLoop.setpoint!.signal_id, (event.currentTarget as HTMLInputElement).value)} />
              <button class="btn btn-primary" onclick={() => apply(selectedLoop.setpoint!)}>Set</button>
            </div>
          {/if}
          {#if selectedLoop.mode && selectedLoop.mode.access === 'write'}
            <div class="extra-row">
              <span>Mode</span>
              <input value={draftFor(selectedLoop.mode)} oninput={(event) => setDraft(selectedLoop.mode!.signal_id, (event.currentTarget as HTMLInputElement).value)} />
              <button class="btn btn-primary" onclick={() => apply(selectedLoop.mode!)}>Set</button>
            </div>
          {/if}
          <div class="actions">
            <button class="btn btn-primary" disabled={selectedLoop.startAction?.access !== 'invoke'} onclick={() => selectedLoop.startAction && onTriggerAction(selectedLoop.startAction.signal_id)}>Start</button>
            <button class="btn btn-danger" disabled={selectedLoop.stopAction?.access !== 'invoke'} onclick={() => selectedLoop.stopAction && onTriggerAction(selectedLoop.stopAction.signal_id)}>Stop</button>
          </div>
          {#each selectedLoop.extra as signal (signal.signal_id)}
            <div class={`extra-row ${selectedId === signal.signal_id ? 'selected' : ''}`}>
              <button onclick={() => onSelectSignal(signal.signal_id)}>{title(leaf(signal.path))}</button>
              {#if signal.signal_type === 'bool'}
                <label><input type="checkbox" checked={bool(draftFor(signal))} oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).checked ? 'true' : 'false')} /> {bool(draftFor(signal)) ? 'true' : 'false'}</label>
              {:else}
                <input value={draftFor(signal)} oninput={(event) => setDraft(signal.signal_id, (event.currentTarget as HTMLInputElement).value)} />
              {/if}
              <button class="btn btn-primary" onclick={() => apply(signal)}>Set</button>
            </div>
          {/each}
        </div>
      {/if}
    </section>
  </div>
</section>

<style>
  .control-tuner { display: grid; grid-template-rows: auto auto 1fr; gap: 0.55rem; min-height: 0; padding: 0.72rem; }
  header h2 { margin: 0; font-size: 0.98rem; color: var(--text-strong); }
  header p { margin: 0.2rem 0 0; font-size: 0.75rem; color: var(--text-soft); }
  .toolbar { display: flex; gap: 0.45rem; align-items: center; }
  .toolbar input { flex: 1 1 auto; min-width: 0; }
  .toolbar span { font-size: 0.68rem; color: var(--text-soft); border: 1px solid var(--border-subtle); border-radius: 999px; padding: 0.2rem 0.45rem; background: var(--surface-3); }
  .layout { min-height: 0; display: grid; grid-template-columns: minmax(240px, 320px) 1fr; gap: 0.5rem; }
  .loop-list { min-height: 0; overflow: auto; display: grid; gap: 0.28rem; align-content: start; padding: 0.48rem; }
  .loop-item { border: 1px solid var(--border-subtle); border-radius: 8px; background: var(--surface-3); padding: 0.34rem 0.4rem; text-align: left; color: inherit; display: grid; gap: 0.1rem; }
  .loop-item strong { font-size: 0.72rem; color: var(--text-strong); }
  .loop-item small { font-size: 0.62rem; color: var(--text-soft); }
  .loop-item code { font-size: 0.6rem; color: var(--text-soft); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .loop-item.active { border-color: rgba(180, 35, 45, 0.62); background: rgba(180, 35, 45, 0.2); }
  .detail { min-height: 0; overflow: auto; display: grid; gap: 0.38rem; align-content: start; padding: 0.5rem; }
  .loop-head h3 { margin: 0; font-size: 0.82rem; color: var(--text-strong); }
  .loop-head code { font-size: 0.63rem; color: var(--text-soft); font-family: var(--font-mono); }
  .metrics { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 0.3rem; }
  .metric { border: 1px solid var(--border-subtle); border-radius: 8px; background: var(--surface-2); color: inherit; text-align: left; padding: 0.3rem 0.38rem; display: grid; gap: 0.14rem; }
  .metric span { font-size: 0.62rem; color: var(--text-soft); text-transform: uppercase; letter-spacing: 0.04em; }
  .metric strong { font-size: 0.74rem; color: var(--text-strong); font-family: var(--font-mono); }
  .metric.selected { border-color: rgba(180, 35, 45, 0.62); }
  .reach { display: flex; justify-content: space-between; align-items: center; border: 1px solid var(--border-subtle); border-radius: 8px; padding: 0.34rem 0.42rem; background: var(--surface-2); }
  .reach strong { font-size: 0.72rem; color: var(--text-strong); }
  .reach p { margin: 0.08rem 0 0; font-size: 0.66rem; color: var(--text-soft); }
  .reach label { display: grid; gap: 0.12rem; font-size: 0.62rem; color: var(--text-soft); }
  .reach input { width: 6rem; font-size: 0.66rem; padding: 0.2rem 0.3rem; font-family: var(--font-mono); }
  .reach[data-state='ok'] { border-color: rgba(52, 211, 153, 0.6); background: rgba(6, 78, 59, 0.2); }
  .reach[data-state='off'] { border-color: rgba(248, 113, 113, 0.56); background: rgba(127, 29, 29, 0.2); }
  .main-grid { display: grid; grid-template-columns: 1.2fr 1fr; gap: 0.42rem; min-height: 0; }
  .graph, .params, .extras { border: 1px solid var(--border-subtle); border-radius: 10px; background: var(--surface-2); padding: 0.38rem; min-height: 0; }
  .graph svg { width: 100%; min-height: 9rem; border: 1px solid rgba(99, 115, 140, 0.44); border-radius: 8px; background: #141a25; }
  .graph path { fill: none; stroke-width: 1.25; vector-effect: non-scaling-stroke; }
  .guide { stroke: rgba(153, 164, 180, 0.18); stroke-width: 0.45; }
  .legend { margin-top: 0.26rem; display: grid; gap: 0.2rem; }
  .legend-item { border: 1px solid var(--border-subtle); border-radius: 7px; background: var(--surface-3); color: inherit; display: grid; grid-template-columns: auto 1fr auto; gap: 0.3rem; align-items: center; padding: 0.2rem 0.3rem; text-align: left; }
  .legend-item i { width: 0.46rem; height: 0.46rem; border-radius: 999px; display: block; }
  .legend-item span { font-size: 0.64rem; color: var(--text); }
  .legend-item b { font-size: 0.64rem; color: var(--text-strong); font-family: var(--font-mono); }
  .legend-item.selected { border-color: rgba(180, 35, 45, 0.62); }
  .params header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.26rem; }
  .params strong { font-size: 0.75rem; color: var(--text-strong); }
  .param-row, .extra-row { border: 1px solid var(--border-subtle); border-radius: 7px; background: var(--surface-3); display: grid; grid-template-columns: minmax(80px, auto) 1fr auto; gap: 0.24rem; align-items: center; padding: 0.22rem; margin-bottom: 0.2rem; }
  .param-row.selected, .extra-row.selected { border-color: rgba(180, 35, 45, 0.62); }
  .param-row button:first-child, .extra-row button:first-child { border: 0; background: transparent; color: var(--text-soft); text-align: left; font-size: 0.66rem; padding: 0; }
  .param-row input, .extra-row input { min-width: 0; font-size: 0.66rem; padding: 0.2rem 0.28rem; font-family: var(--font-mono); }
  .extras { display: grid; gap: 0.2rem; align-content: start; }
  .actions { display: inline-flex; gap: 0.24rem; margin-bottom: 0.14rem; }
  .empty { margin: 0; font-size: 0.72rem; color: var(--text-soft); border: 1px dashed var(--border-emphasis); border-radius: 8px; background: var(--surface-2); padding: 0.55rem; }
  @media (max-width: 1200px) { .main-grid { grid-template-columns: 1fr; } .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
  @media (max-width: 980px) { .layout { grid-template-columns: 1fr; grid-template-rows: minmax(160px, 28vh) minmax(0, 1fr); } }
</style>
