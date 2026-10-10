/**
 * The signal drawer's tree: mechanisms, then their path folders (`Actions`, `Controls/<control>`,
 * `Devices/<device>`, ...), then signals. Folders that stand for one thing (a control loop, a device, a
 * mechanism) carry a `unit` so dragging the folder can create the widget for the whole thing.
 */
import type { SignalDescriptor } from '../data/types';
import { humanLabel } from './widgets/controls-values';

export type TreeUnit = 'mechanism' | 'control' | 'device' | 'folder';

export type TreeNode = {
  /** Unique key: the folder or signal path. */
  key: string;
  label: string;
  /** Set on folders; signals have none. */
  unit: TreeUnit | null;
  /** Signal at a leaf. */
  signal: SignalDescriptor | null;
  children: TreeNode[];
  /** Signals under this node (itself for a leaf). */
  count: number;
};

function folder(key: string, label: string, unit: TreeUnit): TreeNode {
  return { key, label, unit, signal: null, children: [], count: 0 };
}

/** Folder order inside a mechanism: what people reach for first. */
const GROUP_ORDER = ['State', 'Actions', 'Values', 'Controls', 'Devices'];

function sortChildren(node: TreeNode): void {
  node.children.sort((a, b) => {
    const folderFirst = Number(a.signal !== null) - Number(b.signal !== null);
    if (folderFirst !== 0) return folderFirst;
    const ga = GROUP_ORDER.indexOf(a.label);
    const gb = GROUP_ORDER.indexOf(b.label);
    if (ga !== gb) return (ga < 0 ? 99 : ga) - (gb < 0 ? 99 : gb);
    return a.label.localeCompare(b.label);
  });
  for (const child of node.children) {
    if (child.signal === null) sortChildren(child);
  }
}

function countSignals(node: TreeNode): number {
  node.count = node.signal ? 1 : node.children.reduce((sum, c) => sum + countSignals(c), 0);
  return node.count;
}

/**
 * Builds the tree from the dictionary. Lock and system signals are left out. A non-empty `query` keeps
 * only signals whose path contains it (case-insensitive).
 */
export function buildSignalTree(signals: readonly SignalDescriptor[], query = ''): TreeNode[] {
  const needle = query.trim().toLowerCase();
  const roots = new Map<string, TreeNode>();
  for (const signal of signals) {
    if (signal.role === 'lock' || signal.role === 'system') continue;
    if (needle && !signal.path.toLowerCase().includes(needle)) continue;
    const rootKey = signal.mechanismPath || signal.path.split('/').slice(0, 2).join('/');
    let node = roots.get(rootKey);
    if (!node) {
      const name = rootKey.replace(/^robot\//, '').split('/').pop() ?? rootKey;
      node = folder(rootKey, humanLabel(name), signal.mechanismPath ? 'mechanism' : 'folder');
      roots.set(rootKey, node);
    }
    const local = signal.mechanismPath ? signal.localPath : signal.path.slice(rootKey.length + 1) || signal.name;
    const parts = local.split('/');
    let parent = node;
    for (let i = 0; i < parts.length - 1; i++) {
      const key = `${rootKey}/${parts.slice(0, i + 1).join('/')}`;
      let next = parent.children.find((c) => c.key === key && c.signal === null);
      if (!next) {
        const unit: TreeUnit = i === 1 && parts[0] === 'Controls' ? 'control' : i === 1 && parts[0] === 'Devices' ? 'device' : 'folder';
        // Group names stay as written (State, Actions); control and device names read as words.
        next = folder(key, i === 0 ? parts[i] : humanLabel(parts[i]), unit);
        parent.children.push(next);
      }
      parent = next;
    }
    parent.children.push({ key: signal.path, label: humanLabel(signal.name), unit: null, signal, children: [], count: 1 });
  }
  const out = [...roots.values()].sort((a, b) => a.label.localeCompare(b.label));
  for (const root of out) {
    sortChildren(root);
    countSignals(root);
  }
  return out;
}

/** Every signal under a node, depth first. */
export function signalsUnder(node: TreeNode): SignalDescriptor[] {
  return node.signal ? [node.signal] : node.children.flatMap(signalsUnder);
}
