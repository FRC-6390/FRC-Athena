<script lang="ts">
  import { gainLabel } from '../../../lib/mechanisms/model';
  import type { WidgetContext } from '../../../lib/board/types';
  import type { SignalDescriptor } from '../../../lib/data/types';
  import { parseInput, formatNumber } from '../../../lib/format';
  import { controlSiblings } from '../../../lib/board/widgets/robot-siblings';
  import type { ChartSeries } from '../../../lib/chart/time-chart';
  import TimeChart from '../../../ui/TimeChart.svelte';

  let { ctx }: { ctx: WidgetContext } = $props();

  const anchor = $derived(ctx.bindings.signal?.[0]);
  const windowSeconds = $derived(Math.max(1, ctx.setting<number>('window')));
  const showGains = $derived(ctx.setting<boolean>('showGains') !== false);

  const ctrl = $derived.by(() => {
    void ctx.live.current;
    if (!anchor) return null;
    return controlSiblings(ctx.session.store.allSignals(), anchor.path);
  });

  /** Target, reference and measured traces over the window, drawn by the shared time chart. */
  const traces = $derived.by((): ChartSeries[] => {
    void ctx.live.current;
    const c = ctrl;
    const defs = [
      { id: 'target', label: 'target', signal: c?.target ?? null, color: 'var(--series-target)', dashed: false },
      { id: 'reference', label: 'reference', signal: c?.reference ?? null, color: 'var(--series-reference)', dashed: true },
      { id: 'measured', label: 'measured', signal: c?.measured ?? null, color: 'var(--series-measured)', dashed: false }
    ];
    return defs.map((d) => ({ id: d.id, label: d.label, color: d.color, dashed: d.dashed, samples: d.signal ? ctx.samples(d.signal) : [] }));
  });

  const velocity = $derived.by(() => {
    void ctx.live.current;
    const s = ctrl?.velocity;
    if (!s) return null;
    const v = ctx.latest(s);
    return typeof v === 'number' ? v : null;
  });

  // Gains: click a value to edit. The robot's reply (accepted or rejected) comes back through the store.
  let editingPath = $state<string | null>(null);
  let draft = $state('');

  async function commit(signal: SignalDescriptor) {
    const parsed = parseInput(signal.type, draft);
    editingPath = null;
    if (parsed !== null) await ctx.write(signal, parsed);
  }

  function rejected(signal: SignalDescriptor): string | null {
    void ctx.live.current;
    const r = ctx.reply(signal);
    return r && r.op === 'write' && !r.accepted ? r.reason : null;
  }

  const fmt = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? formatNumber(v) : '--');
</script>

{#if !anchor}
  <div class="problem">Bind a Controls/ signal</div>
{:else if !ctrl}
  <div class="problem">{anchor.path} is not under a Controls/ group</div>
{:else}
  <div class="tuner">
    {#if velocity !== null}
      <header><span class="vel ui-mono">vel {formatNumber(velocity)}</span></header>
    {/if}

    <div class="plot">
      <TimeChart series={traces} windowSeconds={windowSeconds} />
    </div>

    {#if showGains && ctrl.gains.length > 0}
      <div class="gains">
        {#each ctrl.gains as gain (gain.path)}
          {@const err = rejected(gain)}
          {@const locked = ctx.locked(gain)}
          <div class="gain">
            <span class="gname ui-mono" title={gain.path}>{gainLabel(gain)}</span>
            {#if editingPath === gain.path}
              <!-- svelte-ignore a11y_autofocus -->
              <input
                class="ui-input"
                autofocus
                bind:value={draft}
                onkeydown={(e) => {
                  if (e.key === 'Enter') void commit(gain);
                  if (e.key === 'Escape') editingPath = null;
                }}
                onblur={() => (editingPath = null)}
              />
            {:else}
              <button
                type="button"
                class="gvalue ui-mono"
                disabled={!gain.writable || ctx.editing}
                title={gain.writable ? 'Click to edit' : gain.path}
                onclick={() => {
                  draft = String(ctx.latest(gain) ?? '');
                  editingPath = gain.path;
                }}>{fmt(ctx.latest(gain))}</button
              >
            {/if}
            {#if locked}<span class="ui-badge" data-tone="locked">locked</span>{/if}
            {#if err}<span class="rejected" title={err}>rejected</span>{/if}
          </div>
        {/each}
      </div>
    {/if}
  </div>
{/if}

<style>
  .tuner {
    display: flex;
    flex-direction: column;
    gap: 6px;
    height: 100%;
    padding: 8px 10px;
    box-sizing: border-box;
    min-height: 0;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
  }
  .vel {
    font-size: var(--fs-xs);
    color: var(--text-soft);
  }
  .plot {
    display: flex;
    flex: 1;
    flex-direction: column;
    min-height: 120px;
  }
  .gains {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(96px, 1fr));
    gap: 4px 8px;
    max-height: 90px;
    overflow: auto;
  }
  .gain {
    display: flex;
    align-items: center;
    gap: 6px;
    min-width: 0;
  }
  .gname {
    font-size: var(--fs-xs);
    color: var(--text-faint);
    min-width: 28px;
  }
  .gvalue {
    border: none;
    background: transparent;
    padding: 0 2px;
    font-size: var(--fs-sm);
    font-weight: 600;
    color: var(--text-strong);
    cursor: pointer;
  }
  .gvalue:disabled {
    cursor: default;
    color: var(--text-soft);
  }
  .gain input {
    width: 72px;
    height: 22px;
    font-size: var(--fs-xs);
  }
  .rejected {
    font-size: var(--fs-xs);
    color: var(--fault);
  }
  .problem {
    display: grid;
    place-items: center;
    height: 100%;
    padding: 8px;
    text-align: center;
    font-size: var(--fs-sm);
    color: var(--text-faint);
  }
</style>
