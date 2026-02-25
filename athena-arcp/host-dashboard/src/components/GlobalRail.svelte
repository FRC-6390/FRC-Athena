<script lang="ts">
  import { FontAwesomeIcon } from '@fortawesome/svelte-fontawesome';
  import {
    faBolt,
    faCamera,
    faChartLine,
    faStethoscope,
    faMicrochip,
    faGear,
    faTableCellsLarge,
    type IconDefinition
  } from '@fortawesome/free-solid-svg-icons';
  import athenaSymbol from '../assets/athena-symbol-red.svg';

  type RailSection =
    | 'dashboards'
    | 'signals'
    | 'mechanisms'
    | 'actions'
    | 'diagnostics'
    | 'camera_tuning'
    | 'settings';

  type Props = {
    section: RailSection;
    onActivate: (section: RailSection) => void;
  };

  const railItems: Array<{ key: RailSection; label: string; icon: IconDefinition }> = [
    { key: 'dashboards', label: 'Dashboards', icon: faTableCellsLarge },
    { key: 'signals', label: 'Signals', icon: faChartLine },
    { key: 'mechanisms', label: 'Mechanisms', icon: faMicrochip },
    { key: 'actions', label: 'Actions', icon: faBolt },
    { key: 'diagnostics', label: 'Diagnostics', icon: faStethoscope },
    { key: 'camera_tuning', label: 'Camera Tuning', icon: faCamera },
    { key: 'settings', label: 'Settings', icon: faGear }
  ];

  let { section, onActivate }: Props = $props();
</script>

<aside class="global-rail">
  <div class="rail-logo">
    <img src={athenaSymbol} alt="Athena symbol" />
  </div>
  <nav class="rail-nav" aria-label="Application navigation">
    {#each railItems as item (item.key)}
      <button
        class={`rail-item ${section === item.key ? 'active' : ''}`}
        aria-label={item.label}
        aria-pressed={section === item.key}
        title={item.label}
        onclick={() => onActivate(item.key)}
      >
        <FontAwesomeIcon icon={item.icon} class="rail-icon" />
      </button>
    {/each}
  </nav>
  <div class="rail-footer">ARCP</div>
</aside>

<style>
  .global-rail {
    background: #141924;
    border-right: 1px solid var(--border);
    display: grid;
    grid-template-rows: auto 1fr auto;
    align-items: start;
    gap: 0.7rem;
    padding: 0.55rem 0.38rem;
  }

  .rail-logo {
    width: 2rem;
    height: 2rem;
    border-radius: 6px;
    display: grid;
    place-items: center;
    margin: 0 auto;
    border: 1px solid var(--border-subtle);
    background: #111722;
    padding: 0.06rem;
  }

  .rail-logo img {
    display: block;
    width: 88%;
    height: 88%;
    object-fit: contain;
    object-position: center center;
    margin: auto;
  }

  .rail-nav {
    display: grid;
    gap: 0.3rem;
    justify-items: center;
  }

  .rail-item {
    width: 2rem;
    height: 2rem;
    border-radius: 6px;
    border: 1px solid transparent;
    background: transparent;
    color: var(--text-soft);
    padding: 0;
    display: inline-grid;
    place-items: center;
    transition:
      border-color 120ms ease,
      background-color 120ms ease,
      color 120ms ease;
  }

  .rail-icon {
    width: 1.02rem;
    height: 1.02rem;
    display: block;
  }

  .rail-item:hover,
  .rail-item.active {
    background: rgba(180, 35, 45, 0.22);
    border-color: rgba(180, 35, 45, 0.45);
    color: #fff;
  }

  .rail-item:focus-visible {
    outline: 2px solid rgba(180, 35, 45, 0.56);
    outline-offset: 1px;
  }

  .rail-footer {
    margin: 0 auto;
    font-size: 0.64rem;
    color: var(--text-soft);
    font-family: var(--font-display);
    letter-spacing: 0.06em;
  }

  @media (max-width: 1380px) {
    .global-rail {
      padding: 0.44rem 0.24rem;
      gap: 0.54rem;
    }

    .rail-logo {
      width: 1.76rem;
      height: 1.76rem;
    }

    .rail-item {
      width: 1.84rem;
      height: 1.84rem;
    }

    .rail-icon {
      width: 0.94rem;
      height: 0.94rem;
    }

    .rail-footer {
      font-size: 0.58rem;
      letter-spacing: 0.05em;
    }
  }
</style>
