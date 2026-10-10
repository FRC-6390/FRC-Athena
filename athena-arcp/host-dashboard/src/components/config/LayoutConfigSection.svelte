<script lang="ts">
  import { readLayoutAccordionConfig, readLayoutGridConfig, readLayoutTitleConfig } from '../../lib/widget-config';
  import type { LayoutSectionProps } from './types';

  let { selectedWidget, setConfig }: LayoutSectionProps = $props();

</script>

{#if selectedWidget.kind === 'layout_title'}
  {@const titleCfg = readLayoutTitleConfig(selectedWidget.config, selectedWidget.title || 'Title')}
  <section class="config-block">
    <h4>Title</h4>
    <div class="row-grid">
      <label>
        Text
        <input
          value={titleCfg.text}
          oninput={(event) =>
            setConfig({
              ...titleCfg,
              text: (event.currentTarget as HTMLInputElement).value
            })}
        />
      </label>
      <label>
        Text color
        <input
          type="color"
          value={titleCfg.color.startsWith('#') ? titleCfg.color : '#f5f7fb'}
          oninput={(event) =>
            setConfig({
              ...titleCfg,
              color: (event.currentTarget as HTMLInputElement).value
            })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'layout_accordion'}
  {@const accordionCfg = readLayoutAccordionConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Accordion Layout</h4>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={accordionCfg.collapsed}
        oninput={(event) =>
          setConfig({
            ...accordionCfg,
            collapsed: (event.currentTarget as HTMLInputElement).checked
          })}
      />
      Collapse children
    </label>
    <p class="hint">Collapsed accordions hide all descendant widgets.</p>
  </section>
{:else if selectedWidget.kind === 'layout_grid'}
  {@const layoutColumns = Math.max(1, Math.floor(selectedWidget.layout.w))}
  {@const layoutRows = Math.max(1, Math.floor(selectedWidget.layout.h))}
  {@const gridCfg = readLayoutGridConfig(selectedWidget.config, layoutColumns, layoutRows)}
  {@const effectiveColumns = gridCfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(selectedWidget.layout.w)))
    : gridCfg.columns}
  {@const effectiveRows = gridCfg.autoSize
    ? Math.max(1, Math.min(96, Math.round(selectedWidget.layout.h)))
    : gridCfg.rows}
  <section class="config-block">
    <h4>Grid Layout</h4>
    <label class="row-inline">
      <input
        type="checkbox"
        checked={gridCfg.autoSize}
        oninput={(event) =>
          setConfig({
            ...gridCfg,
            autoSize: (event.currentTarget as HTMLInputElement).checked
          })}
      />
      Auto size to canvas cells
    </label>
    <div class="row-grid">
      <label>
        Columns
        <input
          type="number"
          min="1"
          max="96"
          value={gridCfg.columns}
          disabled={gridCfg.autoSize}
          oninput={(event) =>
            setConfig({
              ...gridCfg,
              columns: Math.max(
                1,
                Math.min(96, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || layoutColumns))
              )
            })}
        />
      </label>
      <label>
        Rows
        <input
          type="number"
          min="1"
          max="96"
          value={gridCfg.rows}
          disabled={gridCfg.autoSize}
          oninput={(event) =>
            setConfig({
              ...gridCfg,
              rows: Math.max(
                1,
                Math.min(96, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || layoutRows))
              )
            })}
        />
      </label>
    </div>
    {#if gridCfg.autoSize}
      <p class="hint">Effective size: {effectiveColumns} columns x {effectiveRows} rows (tracks canvas cell size).</p>
    {:else}
      <p class="hint">Children snap to this grid's rows/columns.</p>
    {/if}
  </section>
{/if}
