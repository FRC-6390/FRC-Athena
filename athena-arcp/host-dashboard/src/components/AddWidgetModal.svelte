<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import { faXmark } from '@fortawesome/free-solid-svg-icons';
  import { tick } from 'svelte';
  import type { SignalRow } from '../lib/arcp';
  import { signalRole } from '../lib/dashboard';
  import type { WidgetKind } from '../lib/dashboard';

  type Props = {
    open: boolean;
    signals: SignalRow[];
    initialSignalId: number | null;
    defaultWidgetKind: (signal: SignalRow) => WidgetKind;
    widgetKindsFor: (signal: SignalRow | null) => WidgetKind[];
    widgetKindLabel: (kind: WidgetKind) => string;
    leafPath: (path: string) => string;
    onClose: () => void;
    onCreate: (signal: SignalRow, kind: WidgetKind, title: string) => void;
  };

  let {
    open,
    signals,
    initialSignalId,
    defaultWidgetKind,
    widgetKindsFor,
    widgetKindLabel,
    leafPath,
    onClose,
    onCreate
  }: Props = $props();

  let query = $state('');
  let selectedSignalId = $state<number | null>(null);
  let kind = $state<WidgetKind>('metric');
  let title = $state('');
  let modalEl = $state<HTMLDivElement | null>(null);

  const signalById = $derived(new Map(signals.map((signal) => [signal.signal_id, signal])));

  const filteredSignals = $derived.by(() => {
    const q = query.trim().toLowerCase();
    if (!q) return signals;
    return signals.filter((signal) => {
      const role = signalRole(signal);
      return (
        signal.path.toLowerCase().includes(q) ||
        role.toLowerCase().includes(q) ||
        signal.kind.toLowerCase().includes(q) ||
        signal.access.toLowerCase().includes(q) ||
        signal.signal_type.toLowerCase().includes(q) ||
        String(signal.signal_id).includes(q)
      );
    });
  });

  const selectedSignal = $derived(
    selectedSignalId === null ? null : signalById.get(selectedSignalId) ?? null
  );

  const availableKinds = $derived(widgetKindsFor(selectedSignal));

  $effect(() => {
    if (!open) return;

    query = '';
    const preferred =
      initialSignalId !== null && signalById.has(initialSignalId)
        ? initialSignalId
        : signals.length > 0
          ? signals[0].signal_id
          : null;

    selectedSignalId = preferred;

    const signal = preferred !== null ? signalById.get(preferred) ?? null : null;
    if (signal) {
      kind = defaultWidgetKind(signal);
      title = `${leafPath(signal.path)} · ${widgetKindLabel(kind)}`;
    } else {
      title = '';
    }

    void tick().then(() => modalEl?.focus());
  });

  $effect(() => {
    if (!open) return;
    const onKeydown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onClose();
      }
    };
    window.addEventListener('keydown', onKeydown);
    return () => window.removeEventListener('keydown', onKeydown);
  });

  $effect(() => {
    if (!selectedSignal) return;
    if (!availableKinds.includes(kind)) {
      kind = availableKinds[0] ?? defaultWidgetKind(selectedSignal);
    }
  });

  function selectSignal(signal: SignalRow) {
    selectedSignalId = signal.signal_id;
    kind = defaultWidgetKind(signal);
    title = `${leafPath(signal.path)} · ${widgetKindLabel(kind)}`;
  }

  function submitCreate() {
    if (!selectedSignal) return;
    onCreate(selectedSignal, kind, title.trim());
    onClose();
  }

  function onBackdropClick(event: MouseEvent) {
    if (event.target === event.currentTarget) {
      onClose();
    }
  }

</script>

{#if open}
  <div
    class="modal-backdrop"
    role="presentation"
    onclick={onBackdropClick}
  >
    <div
      class="modal"
      bind:this={modalEl}
      role="dialog"
      aria-modal="true"
      aria-label="Add widget"
      tabindex="-1"
    >
      <header class="modal-header">
        <div>
          <h2>Add Panel</h2>
          <p>Select a signal then choose a panel type.</p>
        </div>
        <button class="icon-btn" onclick={onClose} aria-label="Close">
          <FontAwesomeIcon icon={faXmark} class="icon-btn-glyph" />
        </button>
      </header>

      <section class="modal-grid">
        <div class="left">
          <input
            value={query}
            placeholder="Search signal path/type/category/kind/id"
            oninput={(e) => (query = (e.currentTarget as HTMLInputElement).value)}
          />

          <div class="signal-list">
            {#if filteredSignals.length === 0}
              <div class="empty">No matching signals.</div>
            {:else}
              {#each filteredSignals as signal}
                <button
                  type="button"
                  class={`signal-row ${selectedSignalId === signal.signal_id ? 'active' : ''}`}
                  onclick={() => selectSignal(signal)}
                >
                  <strong>{leafPath(signal.path)}</strong>
                  <span>#{signal.signal_id} · {signalRole(signal)} · {signal.signal_type}</span>
                </button>
              {/each}
            {/if}
          </div>
        </div>

        <div class="right">
          {#if selectedSignal}
            <label>
              Panel Type
              <select
                value={kind}
                onchange={(e) => (kind = (e.currentTarget as HTMLSelectElement).value as WidgetKind)}
              >
                {#each availableKinds as option}
                  <option value={option}>{widgetKindLabel(option)}</option>
                {/each}
              </select>
            </label>

            <label>
              Panel Title
              <input value={title} oninput={(e) => (title = (e.currentTarget as HTMLInputElement).value)} />
            </label>

            <div class="preview">
              <h3>Signal Preview</h3>
              <p>{selectedSignal.path}</p>
              <p>{selectedSignal.value}</p>
            </div>

            <button class="btn btn-primary" onclick={submitCreate}>Create Panel</button>
          {:else}
            <p class="empty">Select a signal to continue.</p>
          {/if}
        </div>
      </section>
    </div>
  </div>
{/if}

<style>
  .modal-backdrop {
    position: fixed;
    inset: 0;
    background: rgba(17, 25, 40, 0.34);
    display: grid;
    place-items: center;
    z-index: 60;
    padding: 1rem;
  }

  .modal {
    width: min(1080px, 96vw);
    min-height: min(620px, 90vh);
    border: 1px solid var(--border);
    border-radius: 10px;
    background: var(--surface);
    display: grid;
    gap: 0.58rem;
    padding: 0.64rem;
    outline: none;
  }

  .modal-header {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    gap: 0.5rem;
  }

  .modal-header h2 {
    margin: 0;
    font-size: 0.93rem;
    color: var(--text-strong);
  }

  .modal-header p {
    margin: 0.18rem 0 0;
    color: var(--text-soft);
    font-size: 0.75rem;
  }

  .icon-btn {
    width: 1.42rem;
    height: 1.42rem;
    display: inline-grid;
    place-items: center;
    border-radius: 6px;
    border: 1px solid var(--border-subtle);
    background: var(--surface-3);
    color: var(--text-soft);
    padding: 0;
  }

  .icon-btn-glyph {
    width: 0.66rem;
    height: 0.66rem;
    display: block;
  }

  .modal-grid {
    display: grid;
    grid-template-columns: minmax(300px, 1fr) minmax(250px, 320px);
    gap: 0.58rem;
    min-height: 0;
  }

  .left,
  .right {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.54rem;
    display: grid;
    gap: 0.42rem;
    align-content: start;
  }

  .signal-list {
    max-height: 64vh;
    overflow: auto;
    display: grid;
    gap: 0.35rem;
  }

  .signal-row {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    padding: 0.42rem;
    display: grid;
    gap: 0.2rem;
    cursor: pointer;
    width: 100%;
    text-align: left;
    color: var(--text);
    background: var(--surface);
  }

  .signal-row.active {
    border-color: var(--brand);
    background: var(--brand-soft);
  }

  .signal-row span {
    color: var(--text-soft);
    font-size: 0.72rem;
  }

  label {
    display: grid;
    gap: 0.26rem;
    color: var(--text-soft);
    font-size: 0.74rem;
  }

  .preview {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-3);
    padding: 0.48rem;
    display: grid;
    gap: 0.25rem;
  }

  .preview h3 {
    margin: 0;
    font-size: 0.79rem;
    color: var(--text-strong);
  }

  .preview p {
    margin: 0;
    font-size: 0.75rem;
    color: var(--text-soft);
    word-break: break-word;
  }

  .empty {
    color: var(--text-soft);
    font-size: 0.8rem;
    margin: 0;
  }

  @media (max-width: 900px) {
    .modal-grid {
      grid-template-columns: 1fr;
    }

    .signal-list {
      max-height: 38vh;
    }
  }
</style>
