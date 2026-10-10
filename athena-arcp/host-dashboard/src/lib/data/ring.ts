/** Fixed-capacity ring buffer. Pushing past capacity overwrites the oldest item. */
export class RingBuffer<T> {
  private readonly items: (T | undefined)[];
  private head = 0;
  private count = 0;

  constructor(readonly capacity: number) {
    if (!Number.isInteger(capacity) || capacity < 1) {
      throw new RangeError(`ring capacity must be a positive integer: ${capacity}`);
    }
    this.items = new Array<T | undefined>(capacity);
  }

  get size(): number {
    return this.count;
  }

  push(item: T): void {
    this.items[this.head] = item;
    this.head = (this.head + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }

  /** Newest item, or undefined when empty. */
  latest(): T | undefined {
    if (this.count === 0) return undefined;
    return this.items[(this.head - 1 + this.capacity) % this.capacity];
  }

  /** Items from oldest to newest. */
  toArray(): T[] {
    const out: T[] = [];
    const start = (this.head - this.count + this.capacity) % this.capacity;
    for (let i = 0; i < this.count; i++) {
      out.push(this.items[(start + i) % this.capacity] as T);
    }
    return out;
  }

  clear(): void {
    this.items.fill(undefined);
    this.head = 0;
    this.count = 0;
  }
}
