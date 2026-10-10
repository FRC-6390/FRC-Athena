<script lang="ts">
  import type { ReplayModel } from '../../lib/analysis/replay';

  type Props = {
    model: ReplayModel;
    cursor: number;
    onscrub: (micros: number) => void;
  };

  let { model, cursor, onscrub }: Props = $props();

  const SLIDER_STEPS = 10000;
  const span = $derived(Math.max(1, model.endMicros - model.startMicros));
  const fraction = $derived(Math.min(1, Math.max(0, (cursor - model.startMicros) / span)));
  const ticks = $derived(
    model.markers.map((m) => ({
      key: `${m.timestampMicros}-${m.kind}-${m.message}`,
      offset: Math.min(100, Math.max(0, ((m.timestampMicros - model.startMicros) / span) * 100)),
      label: m.label,
      message: m.message,
      at: m.timestampMicros
    }))
  );

  function onInput(event: Event) {
    const value = Number((event.currentTarget as HTMLInputElement).value) / SLIDER_STEPS;
    onscrub(model.startMicros + value * span);
  }

  function seconds(micros: number): string {
    return `${((micros - model.startMicros) / 1_000_000).toFixed(2)} s`;
  }
</script>

<div class="scrubber">
  <div class="track">
    {#each ticks as tick (tick.key)}
      <button
        type="button"
        class="tick"
        style="left: {tick.offset}%"
        title="{tick.label}: {tick.message} at {seconds(tick.at)}"
        aria-label="jump to {tick.label} at {seconds(tick.at)}"
        onclick={() => onscrub(tick.at)}
      ></button>
    {/each}
    <input
      type="range"
      min="0"
      max={SLIDER_STEPS}
      step="1"
      value={Math.round(fraction * SLIDER_STEPS)}
      oninput={onInput}
      aria-label="replay position"
    />
  </div>
  <div class="readout">
    <span>{seconds(cursor)}</span>
    <span class="muted">of {seconds(model.endMicros)}</span>
  </div>
</div>

<style>
  .scrubber {
    display: grid;
    gap: 4px;
  }
  .track {
    position: relative;
    height: 34px;
  }
  .track input {
    position: absolute;
    left: 0;
    right: 0;
    top: 12px;
    width: 100%;
    margin: 0;
  }
  .tick {
    position: absolute;
    top: 0;
    width: 4px;
    height: 12px;
    margin-left: -2px;
    padding: 0;
    border: 0;
    border-radius: 2px;
    background: #fbbf24;
    cursor: pointer;
    z-index: 1;
  }
  .readout {
    display: flex;
    gap: 10px;
    font-family: var(--font-mono);
    font-size: 13px;
    color: var(--text-strong);
  }
  .muted {
    color: var(--text-soft);
  }
</style>
