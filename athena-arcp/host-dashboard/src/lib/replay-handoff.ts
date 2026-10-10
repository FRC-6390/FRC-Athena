import { writable } from 'svelte/store';

/** A replay file waiting for the Analysis workspace to open it (Pit downloads, then hands it over). */
export const pendingReplay = writable<{ name: string; bytes: Uint8Array } | null>(null);
