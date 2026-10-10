/** Current-draw breakdown for the power split widget. Pure. */

export type PowerEntry = { label: string; amps: number };

export type PowerShare = PowerEntry & {
  /** Share of the total magnitude, 0..1. */
  fraction: number;
};

export type PowerBreakdown = {
  /** Signed sum of all finite entries. */
  total: number;
  /** Sum of absolute values; the stacked bar divides this. */
  magnitude: number;
  shares: PowerShare[];
};

/** Non-finite values count as zero so one bad channel does not blank the whole breakdown. */
export function powerBreakdown(entries: readonly PowerEntry[]): PowerBreakdown {
  const clean = entries.map((e) => ({ label: e.label, amps: Number.isFinite(e.amps) ? e.amps : 0 }));
  const total = clean.reduce((sum, e) => sum + e.amps, 0);
  const magnitude = clean.reduce((sum, e) => sum + Math.abs(e.amps), 0);
  const shares = clean.map((e) => ({ ...e, fraction: magnitude > 0 ? Math.abs(e.amps) / magnitude : 0 }));
  return { total, magnitude, shares };
}
