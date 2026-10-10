<script lang="ts">
  import { onMount } from 'svelte';
  import type { LiveSource } from '../../lib/data/live-source';
  import { lineDiff, type DiffLine } from '../../lib/data/line-diff';
  import { pendingReplay } from '../../lib/replay-handoff';

  type Props = { live: LiveSource | null; onopenanalysis: () => void };
  let { live, onopenanalysis }: Props = $props();

  /** Robot file categories (E6): the server prefixes each name with its category. */
  const CATEGORIES = ['replays', 'store', 'models', 'tuning', 'status'] as const;
  type FileRow = { name: string; size: number; modifiedUnixSecs: number };

  const PROJECT_KEY = 'athena.dashboard.projectDir';
  let category = $state<string>('replays');
  let files = $state<FileRow[]>([]);
  let busy = $state(false);
  let message = $state<string | null>(null);
  let error = $state<string | null>(null);
  let projectDir = $state('');
  let diff = $state<DiffLine[] | null>(null);

  onMount(() => {
    try {
      projectDir = localStorage.getItem(PROJECT_KEY) ?? '';
    } catch {
      /* storage blocked */
    }
    void refresh();
  });

  const shown = $derived(files.filter((f) => f.name.startsWith(`${category}/`)));
  const short = (name: string) => name.slice(name.indexOf('/') + 1);

  async function run<T>(action: () => Promise<T>): Promise<T | undefined> {
    busy = true;
    error = null;
    try {
      return await action();
    } catch (e) {
      error = e instanceof Error ? e.message : String(e);
      return undefined;
    } finally {
      busy = false;
    }
  }

  async function refresh() {
    if (!live) return;
    const list = await run(() => live.fileList());
    if (list) files = list;
  }

  async function download(file: FileRow) {
    if (!live) return;
    const bytes = await run(() => live.fileDownload(file.name));
    if (!bytes) return;
    if (file.name.endsWith('.athrp')) {
      pendingReplay.set({ name: short(file.name), bytes });
      onopenanalysis();
      return;
    }
    // Other files are saved to the user's machine through the browser's download.
    const url = URL.createObjectURL(new Blob([bytes as BlobPart]));
    const link = document.createElement('a');
    link.href = url;
    link.download = short(file.name);
    link.click();
    URL.revokeObjectURL(url);
  }

  async function remove(file: FileRow) {
    if (!live) return;
    if (!confirm(`Delete ${file.name} from the robot? This cannot be undone.`)) return;
    const ok = await run(() => live.fileDelete(file.name));
    if (ok !== undefined) {
      message = `Deleted ${file.name}`;
      await refresh();
    }
  }

  function saveProject() {
    try {
      localStorage.setItem(PROJECT_KEY, projectDir);
    } catch {
      /* storage blocked */
    }
  }

  async function pull() {
    if (!live || !projectDir) return;
    saveProject();
    const path = await run(() => live.pullTuning(projectDir));
    if (path) message = `Saved tuning to ${path}`;
  }

  async function showDiff() {
    if (!live || !projectDir) return;
    saveProject();
    const pair = await run(() => live.tuningDiff(projectDir));
    if (!pair) return;
    if (pair.robot === null) {
      error = 'The robot has no tuning/pull.txt yet';
      diff = null;
      return;
    }
    diff = lineDiff(pair.project ?? '', pair.robot);
  }

  const size = (bytes: number) => (bytes > 1024 * 1024 ? `${(bytes / 1048576).toFixed(1)} MB` : `${Math.ceil(bytes / 1024)} KB`);
</script>

<section class="pit-files">
  {#if !live}
    <p class="note">Connect to a robot to list its files. The mock source has no files.</p>
  {:else}
    <div class="toolbar">
      {#each CATEGORIES as cat (cat)}
        <button type="button" class:active={category === cat} onclick={() => (category = cat)}>{cat}</button>
      {/each}
      <button type="button" onclick={refresh} disabled={busy}>Refresh</button>
    </div>

    <table>
      <thead><tr><th>File</th><th>Size</th><th>Modified</th><th></th></tr></thead>
      <tbody>
        {#each shown as file (file.name)}
          <tr>
            <td>{short(file.name)}</td>
            <td>{size(file.size)}</td>
            <td>{file.modifiedUnixSecs ? new Date(file.modifiedUnixSecs * 1000).toLocaleString() : '-'}</td>
            <td class="actions">
              <button type="button" onclick={() => download(file)} disabled={busy}>
                {file.name.endsWith('.athrp') ? 'Open in Analysis' : 'Download'}
              </button>
              <button type="button" class="danger" onclick={() => remove(file)} disabled={busy}>Delete</button>
            </td>
          </tr>
        {:else}
          <tr><td colspan="4">No files in {category}</td></tr>
        {/each}
      </tbody>
    </table>

    <section class="tuning">
      <h3>Tuning</h3>
      <p class="note">
        Pull saves the robot's <code>tuning/pull.txt</code> to <code>src/main/deploy/athena/tuning/pull.txt</code>
        inside the project folder you choose.
      </p>
      <div class="row">
        <input type="text" bind:value={projectDir} placeholder="/path/to/Rebuilt-2026" aria-label="Project folder" />
        <button type="button" onclick={pull} disabled={busy || !projectDir}>Pull to project</button>
        <button type="button" onclick={showDiff} disabled={busy || !projectDir}>Show diff</button>
      </div>
      {#if diff}
        <pre class="diff">{#each diff as line, i (i)}<span class={line.kind}>{line.kind === 'added' ? '+' : line.kind === 'removed' ? '-' : ' '} {line.text}
</span>{/each}</pre>
      {/if}
    </section>
  {/if}

  {#if message}<p class="ok">{message}</p>{/if}
  {#if error}<p class="error">{error}</p>{/if}
</section>

<style>
  .pit-files {
    display: grid;
    gap: 12px;
    color: var(--text);
  }
  .toolbar,
  .row,
  .actions {
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    align-items: center;
  }
  button.active {
    font-weight: 700;
  }
  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 13px;
  }
  th,
  td {
    text-align: left;
    padding: 6px 8px;
    border-bottom: 1px solid var(--border);
  }
  .tuning h3 {
    margin: 8px 0 4px;
  }
  input[type='text'] {
    min-width: 280px;
    flex: 1;
  }
  .note {
    color: var(--text-soft);
    font-size: 13px;
  }
  .diff {
    margin: 0;
    padding: 8px;
    max-height: 360px;
    overflow: auto;
    background: rgba(0, 0, 0, 0.25);
    font-size: 12px;
  }
  .diff .added {
    color: #34d399;
  }
  .diff .removed {
    color: #f87171;
  }
  .danger {
    color: #f87171;
  }
  .ok {
    color: #34d399;
  }
  .error {
    color: #f87171;
  }
</style>
