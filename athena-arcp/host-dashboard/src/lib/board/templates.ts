import type { SignalDescriptor } from '../data/types';
import { buildMechanismTree, flattenMechanisms, type MechanismNode } from '../data/tree';
import { controlSections, deviceSections, actionSignals, summarize } from '../mechanisms/model';
import { ROBOT_PATHS } from '../data/types';
import { defaultSettings, newId, widgetDef } from './registry';
import type { BoardDoc, WidgetInstance } from './types';

/**
 * Ready-made boards built from the live dictionary, so a team starts from something useful instead of a
 * blank grid. Templates only use signals that exist; unknown widget types are skipped.
 */

const COLUMNS = 24;

class Packer {
  readonly widgets: WidgetInstance[] = [];
  private x = 0;
  private y = 0;
  private rowHeight = 0;

  add(type: string, bindings: Record<string, string[]>, size?: { w: number; h: number }, settings: Record<string, unknown> = {}, title?: string): void {
    const def = widgetDef(type);
    if (!def) return;
    const w = Math.min(COLUMNS, size?.w ?? def.defaultSize.w);
    const h = size?.h ?? def.defaultSize.h;
    if (this.x + w > COLUMNS) this.newRow();
    this.widgets.push({
      id: newId(),
      type,
      x: this.x,
      y: this.y,
      w,
      h,
      ...(title ? { title } : {}),
      bindings,
      settings: { ...defaultSettings(def.settings), ...settings }
    });
    this.x += w;
    this.rowHeight = Math.max(this.rowHeight, h);
  }

  newRow(): void {
    if (this.x === 0) return;
    this.y += this.rowHeight;
    this.x = 0;
    this.rowHeight = 0;
  }
}

function mechanisms(signals: readonly SignalDescriptor[]): MechanismNode[] {
  const roots = buildMechanismTree(signals);
  const robot = roots.find((r) => r.name === 'robot');
  return flattenMechanisms(robot ? robot.children : roots).filter((n) => n.signals.length > 0);
}

const path = (signals: readonly SignalDescriptor[], p: string) => (signals.some((s) => s.path === p) ? [p] : []);

/** Match-day board: clock, battery, alerts, field, auto chooser, camera, and one status card per mechanism. */
export function driverBoard(signals: readonly SignalDescriptor[], name = 'Driver'): BoardDoc {
  const pack = new Packer();
  const poses = signals.filter((s) => s.type === 'struct:Pose2d').map((s) => s.path);
  pack.add('match-timer', { time: path(signals, ROBOT_PATHS.matchTime) }, { w: 5, h: 3 });
  pack.add('match-battery', { volts: path(signals, ROBOT_PATHS.batteryVolts) }, { w: 5, h: 3 });
  pack.add('match-alerts', { faults: [] }, { w: 6, h: 3 });
  const chooser = signals.find((s) => s.type === 'string' && s.writable && /auto/i.test(s.path));
  if (chooser) pack.add('match-chooser', { selected: [chooser.path] }, { w: 8, h: 3 });
  pack.newRow();
  if (poses.length > 0) pack.add('match-field', { pose: [poses[0]] }, { w: 12, h: 7 });
  pack.add('match-camera', {}, { w: 12, h: 7 });
  pack.newRow();
  for (const node of mechanisms(signals)) {
    const s = summarize(node);
    const anchor = s.reason ?? s.state ?? node.signals[0];
    if (anchor) pack.add('mechanismCard', { signal: [anchor.path] }, { w: 6, h: 3 });
  }
  return { id: newId('b'), name, columns: COLUMNS, widgets: pack.widgets };
}

/** Programmer board for one mechanism: tuner per control, gains, actions, devices, values. */
export function tuningBoard(signals: readonly SignalDescriptor[], mechanismPath: string): BoardDoc | null {
  const node = mechanisms(signals).find((n) => n.path === mechanismPath);
  if (!node) return null;
  const pack = new Packer();
  const summary = summarize(node);
  const anchor = summary.reason ?? summary.state ?? node.signals[0];
  if (anchor) pack.add('mechanismCard', { signal: [anchor.path] }, { w: 8, h: 3 });
  const actions = actionSignals(node).filter((a) => !a.localPath.includes('/sysId/'));
  if (actions.length > 0) pack.add('button-group', { actions: actions.map((a) => a.path) }, { w: 16, h: 3 }, {}, 'Actions');
  pack.newRow();
  for (const control of controlSections(node)) {
    const traces = control.plot.length > 0 ? control.plot : control.gains;
    if (traces[0]) pack.add('controlTuner', { signal: [traces[0].path] }, { w: 14, h: 7 });
    if (control.gains.length > 0) pack.add('pid-tuner', { gains: control.gains.map((g) => g.path) }, { w: 10, h: 7 });
    pack.newRow();
  }
  for (const device of deviceSections(node)) {
    const anchorSignal = device.current ?? device.signals[0];
    if (anchorSignal) pack.add('motorCard', { signal: [anchorSignal.path] }, { w: 6, h: 4 });
  }
  pack.newRow();
  const values = node.signals.filter((s) => s.localPath.startsWith('Values/') && (s.type === 'f64' || s.type === 'i64' || s.type === 'bool'));
  if (values.length > 0) pack.add('status-list', { rows: values.slice(0, 12).map((v) => v.path) }, { w: 8, h: Math.min(8, 2 + values.length) }, {}, 'Values');
  const numeric = values.filter((v) => v.type !== 'bool').slice(0, 4);
  if (numeric.length > 0) pack.add('graph', { series: numeric.map((v) => v.path) }, { w: 16, h: 6 }, {}, 'Trends');
  return { id: newId('b'), name: `Tune ${node.name}`, columns: COLUMNS, widgets: pack.widgets };
}

/** Mechanisms that have something worth tuning, for the template menu. */
export function tunableMechanisms(signals: readonly SignalDescriptor[]): { path: string; name: string }[] {
  return mechanisms(signals)
    .filter((n) => controlSections(n).length > 0 || actionSignals(n).length > 0)
    .map((n) => ({ path: n.path, name: n.name }));
}
