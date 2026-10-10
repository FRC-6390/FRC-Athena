import type { ValueStore } from './store';

/** UI refresh period. 30 Hz keeps 50 Hz frames from thrashing rendering. */
export const LIVE_REFRESH_MS = 33;

/**
 * Reactive view of a store for components. Call during component initialisation. Store notifications
 * are coalesced into one refresh per LIVE_REFRESH_MS. A timer is used rather than requestAnimationFrame so
 * updates keep flowing in hidden or background panes. Read `live.current` inside `$derived` to make a
 * computation depend on store changes.
 */
export function useLiveStore(store: ValueStore): { readonly current: number } {
  let version = $state(store.version);
  $effect(() => {
    let scheduled: ReturnType<typeof setTimeout> | null = null;
    const off = store.subscribe(() => {
      if (scheduled !== null) return;
      scheduled = setTimeout(() => {
        scheduled = null;
        version = store.version;
      }, LIVE_REFRESH_MS);
    });
    version = store.version;
    return () => {
      off();
      if (scheduled !== null) clearTimeout(scheduled);
    };
  });
  return {
    get current() {
      return version;
    }
  };
}
