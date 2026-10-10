/**
 * Text helpers for the layout widgets (label, note, image). Pure.
 *
 * Settings are single-line text inputs, so multi-line text is written with the two-character escape `\n`.
 */

/** Turns literal `\n` escapes into line breaks. */
export function unescapeLines(text: string): string {
  return text.replace(/\\n/g, '\n');
}

export type InlineSegment = { text: string; bold: boolean };

/** Splits `**bold**` runs out of a line of text. Unclosed markers are kept as plain text. */
export function parseInline(text: string): InlineSegment[] {
  const out: InlineSegment[] = [];
  const parts = text.split('**');
  // Odd-indexed parts sit between a pair of markers. An unpaired marker leaves the last part as plain.
  parts.forEach((part, i) => {
    if (part === '') return;
    const bold = i % 2 === 1 && i < parts.length - 1;
    if (i % 2 === 1 && i === parts.length - 1) out.push({ text: `**${part}`, bold: false });
    else out.push({ text: part, bold });
  });
  return out;
}

export type NoteBlock =
  | { kind: 'heading'; level: 1 | 2 | 3; text: string }
  | { kind: 'paragraph'; text: string }
  | { kind: 'list'; items: string[] };

/**
 * Small markdown subset: `#`, `##`, `###` headings, `- ` or `* ` bullet lists, blank-line separated
 * paragraphs. Bold is handled by {@link parseInline}. Output is data, so the view never uses raw HTML.
 */
export function parseNote(source: string): NoteBlock[] {
  const blocks: NoteBlock[] = [];
  let paragraph: string[] = [];
  let list: string[] | null = null;

  const flushParagraph = () => {
    if (paragraph.length > 0) blocks.push({ kind: 'paragraph', text: paragraph.join(' ') });
    paragraph = [];
  };
  const flushList = () => {
    if (list && list.length > 0) blocks.push({ kind: 'list', items: list });
    list = null;
  };

  for (const raw of unescapeLines(source).split(/\r?\n/)) {
    const line = raw.trim();
    const heading = /^(#{1,3})\s+(.*)$/.exec(line);
    const bullet = /^[-*]\s+(.*)$/.exec(line);
    if (line === '') {
      flushParagraph();
      flushList();
    } else if (heading) {
      flushParagraph();
      flushList();
      blocks.push({ kind: 'heading', level: heading[1].length as 1 | 2 | 3, text: heading[2] });
    } else if (bullet) {
      flushParagraph();
      list = list ?? [];
      list.push(bullet[1]);
    } else {
      flushList();
      paragraph.push(line);
    }
  }
  flushParagraph();
  flushList();
  return blocks;
}

/** Items of a `|` separated list (checklists), with blanks removed. */
export function parseItems(text: string): string[] {
  return unescapeLines(text)
    .split(/\||\n/)
    .map((item) => item.trim())
    .filter((item) => item !== '');
}

/** Accepts http(s) URLs, data image URIs and site-relative paths. Anything else (javascript:, etc.) is refused. */
export function safeImageUrl(url: string): string | null {
  const trimmed = url.trim();
  if (trimmed === '') return null;
  if (/^https?:\/\//i.test(trimmed)) return trimmed;
  if (/^data:image\/(png|jpe?g|gif|webp|svg\+xml);/i.test(trimmed)) return trimmed;
  if (trimmed.startsWith('/') && !trimmed.startsWith('//')) return trimmed;
  return null;
}
