<script lang="ts" module>
  export const WORKSPACES = ['driver', 'pit', 'tuning', 'analysis'] as const;
  export type Workspace = (typeof WORKSPACES)[number];
</script>

<script lang="ts">
  const LABELS: Record<Workspace, string> = {
    driver: 'Driver',
    pit: 'Pit',
    tuning: 'Tuning',
    analysis: 'Analysis'
  };

  type Props = {
    active: Workspace;
    onselect: (workspace: Workspace) => void;
  };

  let { active, onselect }: Props = $props();
</script>

<div class="switcher" role="tablist" aria-label="workspaces">
  {#each WORKSPACES as workspace (workspace)}
    <button
      type="button"
      role="tab"
      aria-selected={active === workspace}
      class:active={active === workspace}
      onclick={() => onselect(workspace)}
    >
      {LABELS[workspace]}
    </button>
  {/each}
</div>

<style>
  .switcher {
    display: inline-flex;
    gap: 4px;
    padding: 3px;
    background: var(--bg-2);
    border: 1px solid var(--border);
    border-radius: 10px;
  }
  button {
    background: transparent;
    border: 0;
    color: var(--text-soft);
    padding: 6px 14px;
    border-radius: 7px;
    font: inherit;
    cursor: pointer;
  }
  button:hover {
    color: var(--text);
  }
  button.active {
    background: var(--surface-3);
    color: var(--text-strong);
  }
</style>
