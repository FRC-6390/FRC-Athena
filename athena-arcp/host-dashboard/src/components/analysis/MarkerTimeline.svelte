<script lang="ts">
  import type { Session } from '../../lib/data/session';
  import { formatSeconds } from '../../lib/format';

  type Props = {
    session: Session;
    live: { readonly current: number };
  };

  let { session, live }: Props = $props();

  /** Markers newest first, with positions on a timeline spanning the first to the last marker. */
  const timeline = $derived.by(() => {
    void live.current;
    const markers = session.store.markers();
    if (markers.length === 0) return { rows: [], span: 0, start: 0 };
    const start = markers[0].timestampMicros;
    const end = markers[markers.length - 1].timestampMicros;
    const span = Math.max(1, end - start);
    const rows = markers
      .map((m, index) => ({
        key: index,
        label: m.label,
        message: m.message,
        at: m.timestampMicros,
        offset: ((m.timestampMicros - start) / span) * 100,
        signal: m.signalId !== null ? (session.store.descriptor(m.signalId)?.path ?? `#${m.signalId}`) : null
      }))
      .reverse();
    return { rows, span, start };
  });
</script>

<section class="timeline" aria-label="marker timeline">
  <div class="track" aria-hidden="true">
    {#each timeline.rows as row (row.key)}
      <span class="tick" style="left: {row.offset}%" title="{row.label}: {row.message}"></span>
    {/each}
  </div>
  {#if timeline.rows.length === 0}
    <p class="empty">No markers yet. Boot, match and fault markers appear here as they are received.</p>
  {:else}
    <ol>
      {#each timeline.rows as row (row.key)}
        <li>
          <span class="time">{formatSeconds(row.at - timeline.start)}</span>
          <span class="kind">{row.label}</span>
          <span class="message">{row.message}</span>
          {#if row.signal}<span class="signal">{row.signal}</span>{/if}
        </li>
      {/each}
    </ol>
  {/if}
</section>

<style>
  .timeline {
    display: grid;
    gap: 10px;
  }
  .track {
    position: relative;
    height: 26px;
    background: var(--surface-2);
    border: 1px solid var(--border);
    border-radius: 6px;
  }
  .tick {
    position: absolute;
    top: 4px;
    bottom: 4px;
    width: 2px;
    background: var(--brand);
    transform: translateX(-1px);
  }
  ol {
    list-style: none;
    margin: 0;
    padding: 0;
    display: grid;
    gap: 4px;
  }
  li {
    display: grid;
    grid-template-columns: 70px 140px minmax(0, 1fr) minmax(0, 260px);
    gap: 10px;
    font-size: 12px;
    padding: 4px 8px;
    border-radius: 6px;
    background: var(--surface);
  }
  .time,
  .signal {
    font-family: var(--font-mono);
    color: var(--text-soft);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .kind {
    color: var(--text-strong);
  }
  .message {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .empty {
    color: var(--text-soft);
    font-size: 13px;
  }
  @media (max-width: 720px) {
    li {
      grid-template-columns: 70px minmax(0, 1fr);
    }
    .message,
    .signal {
      grid-column: 2;
    }
  }
</style>
