<script lang="ts">
  import type { DashboardWidget, WidgetLayout } from '../../lib/dashboard';
  import { readLayoutAccordionConfig, readLayoutTitleConfig } from '../../lib/widget-config';

  let {
    widget,
    layout,
    gridCounts,
    onToggleAccordion
  }: {
    widget: DashboardWidget;
    layout: WidgetLayout;
    gridCounts: { columns: number; rows: number } | null;
    onToggleAccordion: (widget: DashboardWidget) => void;
  } = $props();

  const accordionConfig = $derived(widget.kind === 'layout_accordion' ? readLayoutAccordionConfig(widget.config) : null);
  const titleConfig = $derived(widget.kind === 'layout_title' ? readLayoutTitleConfig(widget.config, widget.title) : null);
</script>

{#if widget.kind === 'layout_section'}
  <div class="layout-section-body">
    <div class="layout-section-line"></div>
  </div>
{:else if widget.kind === 'layout_divider'}
  <div class="layout-divider" aria-hidden="true"></div>
{:else if widget.kind === 'layout_list'}
  <div class="layout-list" aria-hidden="true">
    {#each Array.from({ length: Math.max(1, Math.min(10, layout.h)) }) as _, index (`list-${widget.id}-${index}`)}
      <span></span>
    {/each}
  </div>
{:else if widget.kind === 'layout_accordion' && accordionConfig}
  <div class={`layout-accordion ${accordionConfig.collapsed ? 'collapsed' : ''}`}>
    <button
      type="button"
      class="layout-accordion-head layout-accordion-toggle"
      aria-label={`${accordionConfig.collapsed ? 'Expand' : 'Collapse'} ${widget.title}`}
      aria-expanded={!accordionConfig.collapsed}
      onclick={() => onToggleAccordion(widget)}
    >
      <strong>{widget.title}</strong>
      <span class="layout-accordion-chevron">{accordionConfig.collapsed ? '>' : 'v'}</span>
    </button>
    <div class="layout-accordion-body">
      {#if accordionConfig.collapsed}
        <button
          type="button"
          class="layout-accordion-collapsed-fill layout-accordion-toggle"
          aria-label={`Expand ${widget.title}`}
          aria-expanded="false"
          onclick={() => onToggleAccordion(widget)}
        >
          <span class="layout-accordion-collapsed-label">{widget.title}</span>
          <span class="layout-accordion-chevron">></span>
        </button>
      {:else}
        <div class="layout-list">
          {#each Array.from({ length: Math.max(1, Math.min(8, layout.h)) }) as _, index (`accordion-${widget.id}-${index}`)}
            <span></span>
          {/each}
        </div>
      {/if}
    </div>
  </div>
{:else if widget.kind === 'layout_grid' && gridCounts}
  <div
    class="layout-grid"
    style={`--layout-cols:${gridCounts.columns};--layout-rows:${gridCounts.rows};`}
    aria-hidden="true"
  ></div>
{:else if widget.kind === 'layout_spacer'}
  <div class="layout-spacer">Spacer</div>
{:else if widget.kind === 'layout_title' && titleConfig}
  <div class="layout-title" style={`color:${titleConfig.color};`}>{titleConfig.text}</div>
{/if}

<style>
  .layout-section-body {
    width: 100%;
    height: 100%;
    display: flex;
    align-items: center;
  }

  .layout-section-line {
    width: 100%;
    border-top: 1px dashed rgba(180, 35, 45, 0.45);
  }

  .layout-divider {
    width: 100%;
    border-top: 1px solid var(--border-emphasis);
  }

  .layout-list {
    width: 100%;
    height: 100%;
    display: flex;
    flex-direction: column;
    gap: 0.16rem;
    justify-content: flex-start;
  }

  .layout-list span {
    display: block;
    width: 100%;
    flex: 0 0 0.42rem;
    min-height: 0.42rem;
    border-radius: 2px;
    background: rgba(153, 164, 180, 0.2);
    border: 1px solid rgba(153, 164, 180, 0.2);
  }

  .layout-accordion {
    width: 100%;
    height: 100%;
    min-height: 1.2rem;
    border: 1px solid rgba(153, 164, 180, 0.24);
    border-radius: 5px;
    background: rgba(28, 35, 47, 0.72);
    display: grid;
    grid-template-rows: auto minmax(0, 1fr);
    overflow: hidden;
  }

  :global(.widget-card.layout-card.accordion-open) .layout-accordion {
    box-shadow: 0 8px 20px rgba(0, 0, 0, 0.34);
  }

  .layout-accordion-head {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.3rem;
    padding: 0.18rem 0.28rem;
    border-bottom: 1px solid rgba(153, 164, 180, 0.24);
    background: rgba(40, 49, 66, 0.8);
  }

  .layout-accordion-toggle {
    width: 100%;
    border: 0;
    margin: 0;
    text-align: left;
    font: inherit;
    color: inherit;
    cursor: pointer;
  }

  .layout-accordion-toggle:focus-visible {
    outline: 2px solid rgba(138, 183, 255, 0.85);
    outline-offset: -2px;
  }

  .layout-accordion-head strong {
    min-width: 0;
    margin: 0;
    font-size: 0.66rem;
    font-weight: 700;
    line-height: 1.1;
    color: var(--text-strong);
    text-overflow: ellipsis;
    white-space: nowrap;
    overflow: hidden;
  }

  .layout-accordion-chevron {
    font-size: 0.62rem;
    color: var(--text-soft);
    line-height: 1;
  }

  .layout-accordion-body {
    min-height: 0;
    padding: 0.14rem;
    overflow: hidden;
  }

  .layout-accordion.collapsed .layout-accordion-body {
    display: flex;
    align-items: stretch;
    justify-content: stretch;
    padding: 0.08rem;
  }

  .layout-accordion.collapsed {
    grid-template-rows: minmax(0, 1fr);
  }

  .layout-accordion.collapsed .layout-accordion-head {
    display: none;
  }

  .layout-accordion-collapsed-fill {
    flex: 1 1 auto;
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    align-items: center;
    gap: 0.18rem;
    padding: 0.1rem 0.2rem;
    width: 100%;
    height: 100%;
    min-height: 0.36rem;
    border: 1px solid rgba(153, 164, 180, 0.28);
    border-radius: 4px;
    color: var(--text-soft);
    background:
      linear-gradient(180deg, rgba(80, 96, 120, 0.2), rgba(45, 57, 76, 0.3)),
      repeating-linear-gradient(
        90deg,
        rgba(153, 164, 180, 0.14),
        rgba(153, 164, 180, 0.14) 5px,
        transparent 5px,
        transparent 10px
      );
  }

  .layout-accordion-collapsed-fill:focus-visible {
    outline: 2px solid rgba(138, 183, 255, 0.85);
    outline-offset: -2px;
  }

  .layout-accordion-collapsed-label {
    min-width: 0;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    font-size: 0.58rem;
    font-weight: 700;
    color: var(--text-strong);
  }

  .layout-grid {
    width: 100%;
    height: 100%;
    min-height: 1.4rem;
    border: 1px solid rgba(153, 164, 180, 0.2);
    border-radius: 4px;
    background:
      linear-gradient(to right, rgba(153, 164, 180, 0.24) 1px, transparent 1px),
      linear-gradient(to bottom, rgba(153, 164, 180, 0.2) 1px, transparent 1px);
    background-size:
      calc(100% / max(1, var(--layout-cols, 4))) 100%,
      100% calc(100% / max(1, var(--layout-rows, 4)));
    background-position: 0 0, 0 0;
  }

  .layout-spacer {
    width: 100%;
    text-align: center;
    color: var(--text-soft);
    font-size: 0.66rem;
    border: 1px dashed rgba(153, 164, 180, 0.35);
    border-radius: 5px;
    padding: 0.25rem;
  }

  .layout-title {
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    text-align: center;
    font-family: var(--font-display);
    font-size: clamp(1rem, calc(var(--cell) * 0.6), 2.2rem);
    line-height: 1.05;
    font-weight: 800;
    letter-spacing: 0.01em;
    overflow-wrap: anywhere;
    padding: 0.1rem;
  }
</style>
