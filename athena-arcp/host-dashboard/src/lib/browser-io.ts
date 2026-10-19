/** Browser file and keyboard helpers shared by the canvas views, recordings and the app shell. */

export function downloadJsonFile(filename: string, contents: string) {
  const blob = new Blob([contents], { type: 'application/json;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  anchor.style.display = 'none';
  document.body.appendChild(anchor);
  anchor.click();
  document.body.removeChild(anchor);
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

export function downloadBinaryFile(filename: string, bytes: Uint8Array) {
  const blob = new Blob([new Uint8Array(bytes)], { type: 'application/octet-stream' });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  anchor.style.display = 'none';
  document.body.appendChild(anchor);
  anchor.click();
  document.body.removeChild(anchor);
  setTimeout(() => URL.revokeObjectURL(url), 0);
}

/** Opens a file picker and resolves with the chosen file's text, or null when cancelled. */
export async function pickJsonFileText(): Promise<string | null> {
  return new Promise((resolve, reject) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json,application/json';
    input.style.display = 'none';
    document.body.appendChild(input);

    const cleanup = () => {
      input.removeEventListener('change', onChange);
      document.body.removeChild(input);
    };

    const onChange = async () => {
      try {
        const file = input.files?.[0];
        if (!file) {
          cleanup();
          resolve(null);
          return;
        }
        const text = await file.text();
        cleanup();
        resolve(text);
      } catch (err) {
        cleanup();
        reject(err);
      }
    };

    input.addEventListener('change', onChange, { once: true });
    input.click();
  });
}

/** True when a global shortcut should not fire because the user is typing or pressing a control. */
export function shouldIgnoreGlobalHotkeys(target: EventTarget | null): boolean {
  const el = target as HTMLElement | null;
  if (!el) return false;
  const tag = el.tagName.toLowerCase();
  if (tag === 'input' || tag === 'textarea' || tag === 'select' || tag === 'button') {
    return true;
  }
  return el.isContentEditable;
}
