/**
 * Checklist memory. Ticks are stored per list, keyed by an explicit key or a hash of the items, and
 * survive reloads through localStorage. Storage can be missing or blocked, so every access is guarded.
 */

/** 32-bit FNV-1a, enough to tell apart the lists on a dashboard. */
export function hashText(text: string): string {
  let h = 0x811c9dc5;
  for (let i = 0; i < text.length; i += 1) {
    h ^= text.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return (h >>> 0).toString(16).padStart(8, '0');
}

export function checklistKey(storageKey: string, items: readonly string[]): string {
  const id = storageKey.trim() !== '' ? storageKey.trim() : hashText(items.join('|'));
  return `athena.board.checklist.${id}`;
}

/** Checked items by text (not index), so editing the list keeps the ticks on the items that remain. */
export function loadChecked(key: string): string[] {
  try {
    const raw = globalThis.localStorage?.getItem(key);
    if (!raw) return [];
    const parsed: unknown = JSON.parse(raw);
    return Array.isArray(parsed) ? parsed.filter((v): v is string => typeof v === 'string') : [];
  } catch {
    return [];
  }
}

export function saveChecked(key: string, checked: readonly string[]): void {
  try {
    if (checked.length === 0) globalThis.localStorage?.removeItem(key);
    else globalThis.localStorage?.setItem(key, JSON.stringify(checked));
  } catch {
    // Storage unavailable: the ticks simply live for this session.
  }
}
