<script lang="ts" module>
  export type Section = 'dashboards' | 'mechanisms' | 'health' | 'replays' | 'signals' | 'settings';
</script>

<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import {
    faTableCellsLarge,
    faMicrochip,
    faHeartPulse,
    faClockRotateLeft,
    faList,
    faGear,
    type IconDefinition
  } from '@fortawesome/free-solid-svg-icons';
  import athenaSymbol from '../assets/athena-symbol-red.svg';

  type Props = {
    section: Section;
    onselect: (section: Section) => void;
    /** Sections that should show an attention dot (e.g. health with active faults). */
    attention?: Partial<Record<Section, 'warn' | 'fault'>>;
  };

  let { section, onselect, attention = {} }: Props = $props();

  const items: Array<{ key: Section; label: string; icon: IconDefinition }> = [
    { key: 'dashboards', label: 'Dashboards', icon: faTableCellsLarge },
    { key: 'mechanisms', label: 'Mechanisms', icon: faMicrochip },
    { key: 'health', label: 'Health', icon: faHeartPulse },
    { key: 'replays', label: 'Replays', icon: faClockRotateLeft },
    { key: 'signals', label: 'Signals', icon: faList }
  ];
</script>

<nav class="rail" aria-label="Sections">
  <div class="logo"><img src={athenaSymbol} alt="Athena" /></div>
  <div class="items">
    {#each items as item (item.key)}
      <button
        type="button"
        class="item"
        class:active={section === item.key}
        aria-current={section === item.key ? 'page' : undefined}
        title={item.label}
        onclick={() => onselect(item.key)}
      >
        <span class="icon">
          <FontAwesomeIcon icon={item.icon} />
          {#if attention[item.key]}<i class="pip" data-tone={attention[item.key]}></i>{/if}
        </span>
        <span class="label">{item.label}</span>
      </button>
    {/each}
  </div>
  <button
    type="button"
    class="item"
    class:active={section === 'settings'}
    title="Settings"
    onclick={() => onselect('settings')}
  >
    <span class="icon"><FontAwesomeIcon icon={faGear} /></span>
    <span class="label">Settings</span>
  </button>
</nav>

<style>
  .rail {
    display: flex;
    flex-direction: column;
    align-items: stretch;
    gap: var(--sp-1);
    width: var(--rail-width);
    padding: var(--sp-2) 6px;
    background: var(--bg-1);
    border-right: 1px solid var(--border);
  }
  .logo {
    display: grid;
    place-items: center;
    height: 40px;
    margin-bottom: var(--sp-2);
  }
  .logo img {
    width: 26px;
    height: 26px;
  }
  .items {
    display: flex;
    flex-direction: column;
    gap: 2px;
    flex: 1;
  }
  .item {
    position: relative;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 3px;
    padding: 8px 0 6px;
    border: none;
    border-radius: var(--radius);
    background: transparent;
    color: var(--text-faint);
    cursor: pointer;
    font: inherit;
  }
  .item:hover {
    color: var(--text);
    background: var(--surface-2);
  }
  .item.active {
    color: var(--text-strong);
    background: var(--surface-3);
  }
  .item.active::before {
    content: '';
    position: absolute;
    left: -6px;
    top: 10px;
    bottom: 10px;
    width: 3px;
    border-radius: 0 3px 3px 0;
    background: var(--brand);
  }
  .icon {
    position: relative;
    font-size: 16px;
    line-height: 1;
  }
  .pip {
    position: absolute;
    top: -3px;
    right: -6px;
    width: 7px;
    height: 7px;
    border-radius: 50%;
    border: 2px solid var(--bg-1);
    background: var(--warn);
  }
  .pip[data-tone='fault'] {
    background: var(--fault);
  }
  .label {
    font-size: 10px;
    font-weight: 500;
    letter-spacing: 0.01em;
  }
</style>
