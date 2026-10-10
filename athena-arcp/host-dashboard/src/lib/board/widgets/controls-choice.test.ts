import { describe, expect, test } from 'bun:test';
import type { SignalDescriptor } from '../../data/types';
import { chooserScore, choiceList, parseChoices, resolveChoices } from './controls-choice';
import { findRestoreAction, gainScore, isGainName } from './controls-pid';

const sig = (path: string, type: SignalDescriptor['type'], writable: boolean, role: SignalDescriptor['role'] = 'value'): SignalDescriptor => {
  const parts = path.split('/');
  const local = parts.slice(2).join('/');
  return {
    signalId: Math.abs(path.length * 31 + path.charCodeAt(path.length - 1)),
    path,
    type,
    typeTag: 1,
    unit: '',
    role,
    writable,
    persisted: false,
    lockGroup: '',
    structSchemaId: 0,
    mechanismPath: parts.slice(0, 2).join('/'),
    localPath: local,
    name: parts[parts.length - 1]
  };
};

describe('choices', () => {
  test('comma list is trimmed and de-duplicated', () => {
    expect(parseChoices(' a, b ,, a ,c')).toEqual(['a', 'b', 'c']);
    expect(parseChoices(undefined)).toEqual([]);
  });
  test('signal options win when present, else settings', () => {
    expect(resolveChoices(['x', 'y', 'x'], 'a,b')).toEqual(['x', 'y']);
    expect(resolveChoices([], 'a,b')).toEqual(['a', 'b']);
    expect(resolveChoices(undefined, 'a')).toEqual(['a']);
  });
  test('robot value outside the options is kept as extra', () => {
    expect(choiceList(['a', 'b'], 'z')).toEqual({ options: ['z', 'a', 'b'], extra: 'z' });
    expect(choiceList(['a', 'b'], 'a')).toEqual({ options: ['a', 'b'], extra: null });
    expect(choiceList(['a'], undefined)).toEqual({ options: ['a'], extra: null });
  });
  test('scores favour writable pick-one strings', () => {
    expect(chooserScore(sig('robot/auto/Controls/autoChoice', 'string', true))).toBe(65);
    expect(chooserScore(sig('robot/x/Controls/label', 'string', true))).toBe(45);
    expect(chooserScore(sig('robot/auto/Values/options', 'string[]', false))).toBe(25);
    expect(chooserScore(sig('robot/x/Values/n', 'f64', true))).toBe(0);
  });
});

describe('pid', () => {
  test('gain names are recognised case-insensitively', () => {
    expect(isGainName('kP')).toBe(true);
    expect(isGainName('iZone')).toBe(true);
    expect(isGainName('setpoint')).toBe(false);
  });
  test('gain score ranks gains above generic writable numbers', () => {
    expect(gainScore(sig('robot/turret/Controls/PID/kP', 'f64', true))).toBe(85);
    expect(gainScore(sig('robot/turret/Controls/speed', 'f64', true))).toBe(20);
    expect(gainScore(sig('robot/turret/Controls/PID/kP', 'f64', false))).toBe(0);
    expect(gainScore(sig('robot/turret/Controls/PID/kP', 'bool', true))).toBe(0);
  });
  test('finds a sibling Restore action in the parent folder', () => {
    const gains = [sig('robot/turret/Controls/fieldHeading/Config/PID/p', 'f64', true), sig('robot/turret/Controls/fieldHeading/Config/PID/i', 'f64', true)];
    const restore = sig('robot/turret/Controls/fieldHeading/Config/Restore', 'bool', false, 'action');
    expect(findRestoreAction(gains, [restore])).toBe(restore);
    expect(findRestoreAction(gains, [])).toBeUndefined();
  });
  test('ignores a Restore that is not an action', () => {
    const gains = [sig('robot/t/Config/PID/p', 'f64', true)];
    expect(findRestoreAction(gains, [sig('robot/t/Config/Restore', 'bool', false, 'state')])).toBeUndefined();
  });
  test('prefers the closest folder', () => {
    const gains = [sig('robot/t/Config/PID/p', 'f64', true)];
    const near = sig('robot/t/Config/PID/Restore', 'bool', false, 'action');
    const far = sig('robot/t/Config/Restore', 'bool', false, 'action');
    expect(findRestoreAction(gains, [far, near])).toBe(near);
  });
});
