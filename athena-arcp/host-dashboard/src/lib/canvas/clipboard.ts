/** Platform hooks used to copy text. Injected so the copy policy can be tested without a browser. */
export type ClipboardDeps = {
  writeText?: (text: string) => Promise<void>;
  /** Legacy copy path: selects a hidden textarea and runs `document.execCommand('copy')`. */
  execCopy?: (text: string) => boolean;
};

/** Browser implementation of the two copy paths, used when no deps are injected. */
export function browserClipboardDeps(): ClipboardDeps {
  return {
    writeText:
      typeof navigator !== 'undefined' && navigator.clipboard
        ? (text) => navigator.clipboard.writeText(text)
        : undefined,
    execCopy: (text) => {
      if (typeof document === 'undefined') return false;
      const input = document.createElement('textarea');
      input.value = text;
      input.setAttribute('readonly', 'true');
      input.style.position = 'fixed';
      input.style.opacity = '0';
      input.style.pointerEvents = 'none';
      document.body.appendChild(input);
      input.select();
      try {
        return document.execCommand('copy');
      } finally {
        document.body.removeChild(input);
      }
    }
  };
}

/**
 * Copies a value: the async clipboard API first, the legacy path if that fails. Resolves true when the text
 * reached the clipboard.
 */
export async function copyText(text: string, deps: ClipboardDeps = browserClipboardDeps()): Promise<boolean> {
  if (deps.writeText) {
    try {
      await deps.writeText(text);
      return true;
    } catch {
      // Fall through to the legacy path where the async API is unavailable or refused.
    }
  }
  return deps.execCopy ? deps.execCopy(text) : false;
}
