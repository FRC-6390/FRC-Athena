<script lang="ts">
  import type { WidgetContext } from '../../../lib/board/types';
  import { retryDelayMs, streamSource, streamUrlProblem } from '../../../lib/board/widgets/match-camera';

  let { ctx }: { ctx: WidgetContext } = $props();

  const urlSignal = $derived(ctx.bindings.url?.[0]);
  const typedUrl = $derived(ctx.setting<string>('url'));
  const crosshair = $derived(ctx.setting<boolean>('crosshair'));
  const fit = $derived(ctx.setting<string>('fit'));

  /** Typed URL wins; a bound string signal (e.g. Vision/stream_url) is the fallback. */
  const url = $derived.by(() => {
    void ctx.live.current;
    if (typedUrl.trim() !== '') return typedUrl;
    const bound = urlSignal ? ctx.latest(urlSignal) : undefined;
    return typeof bound === 'string' ? bound : '';
  });
  const problem = $derived(streamUrlProblem(url));

  let attempt = $state(0);
  let live = $state(false);
  let retryAt = $state<number | null>(null);
  let now = $state(Date.now());

  // Reset the stream when the address changes.
  $effect(() => {
    void url;
    attempt = 0;
    live = false;
    retryAt = null;
  });

  // A failed stream retries with backoff; the timer is cleared with the widget.
  $effect(() => {
    if (retryAt === null) return;
    const wait = Math.max(0, retryAt - Date.now());
    const handle = setTimeout(() => {
      retryAt = null;
      attempt += 1;
    }, wait);
    const tick = setInterval(() => (now = Date.now()), 250);
    return () => {
      clearTimeout(handle);
      clearInterval(tick);
    };
  });

  function onError() {
    live = false;
    retryAt = Date.now() + retryDelayMs(attempt);
    now = Date.now();
  }

  const secondsLeft = $derived(retryAt === null ? 0 : Math.max(0, Math.ceil((retryAt - now) / 1000)));
  const src = $derived(problem === null ? streamSource(url, attempt) : '');
</script>

<div class="camera">
  {#if problem !== null}
    <div class="message">{problem}</div>
  {:else}
    {#key src}
      <img
        class="feed"
        class:cover={fit === 'cover'}
        {src}
        alt="Camera stream"
        onload={() => {
          live = true;
          retryAt = null;
        }}
        onerror={onError}
      />
    {/key}
    {#if crosshair}
      <svg class="crosshair" viewBox="0 0 100 100" preserveAspectRatio="xMidYMid meet" aria-hidden="true">
        <line x1="50" x2="50" y1="0" y2="100" />
        <line x1="0" x2="100" y1="50" y2="50" />
        <circle cx="50" cy="50" r="6" />
      </svg>
    {/if}
    {#if !live}
      <div class="message overlay">
        {retryAt === null ? 'Connecting' : `No signal. Retrying in ${secondsLeft} s`}
      </div>
    {/if}
  {/if}
</div>

<style>
  .camera {
    position: relative;
    width: 100%;
    height: 100%;
    display: grid;
    place-items: center;
    background: var(--bg-0);
    overflow: hidden;
  }
  .feed {
    width: 100%;
    height: 100%;
    object-fit: contain;
    display: block;
  }
  .feed.cover {
    object-fit: cover;
  }
  .crosshair {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
  }
  .crosshair line,
  .crosshair circle {
    stroke: var(--text-strong);
    stroke-width: 0.5;
    fill: none;
    opacity: 0.85;
    vector-effect: non-scaling-stroke;
  }
  .message {
    padding: 12px;
    text-align: center;
    font-size: min(max(12px, 6cqh), 22px);
    font-weight: 600;
    color: var(--text-soft);
  }
  .message.overlay {
    position: absolute;
    left: 0;
    right: 0;
    bottom: 0;
    background: var(--surface-overlay, rgba(10, 12, 15, 0.7));
    color: var(--text-strong);
    font-size: var(--fs-sm);
    padding: 6px;
  }
</style>
