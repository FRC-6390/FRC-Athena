import { SignalRow } from '../arcp';
import { WidgetConfigRecord } from '../dashboard';
import { clamp, findSignalByLeaf, isObject, looksLikeUrl, parentPath, resolveConfiguredSignalId, toBoolean, toNumber, toString } from './common';

export type CameraOverlayConfig = {
  streamSignalId: number | null;
  streamUrl: string;
  poseXSignalId: number | null;
  poseYSignalId: number | null;
  headingSignalId: number | null;
  targetsSignalId: number | null;
  detectionsSignalId: number | null;
  showPose: boolean;
  showTargets: boolean;
  showDetections: boolean;
  mirrorX: boolean;
  sourceWidth: number;
  sourceHeight: number;
};

export type CameraOverlayBox = {
  x: number;
  y: number;
  w: number;
  h: number;
  label: string;
  score: number | null;
};

export function defaultCameraOverlayConfig(anchor: SignalRow, signals: SignalRow[]): CameraOverlayConfig {
  const anchorParent = parentPath(anchor.path);
  const streamSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['stream', 'streamurl', 'url', 'camera', 'video', 'mjpeg'],
    ['string', 'string[]']
  );
  const targetsSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['targets', 'targetboxes', 'target', 'boxes'],
    ['string', 'string[]']
  );
  const detectionsSignal = findSignalByLeaf(
    signals,
    anchorParent,
    ['detections', 'objects', 'visionobjects', 'visionboxes'],
    ['string', 'string[]']
  );
  const poseX = findSignalByLeaf(signals, anchorParent, ['x', 'posex', 'robotx'], ['f64', 'i64']);
  const poseY = findSignalByLeaf(signals, anchorParent, ['y', 'posey', 'roboty'], ['f64', 'i64']);
  const heading = findSignalByLeaf(
    signals,
    anchorParent,
    ['heading', 'yaw', 'theta', 'rotation', 'angle', 'gyro'],
    ['f64', 'i64']
  );

  const streamFromAnchor =
    (anchor.signal_type === 'string' || anchor.signal_type === 'string[]') && looksLikeUrl(anchor.value)
      ? anchor.value.trim()
      : '';

  return {
    streamSignalId:
      streamSignal?.signal_id ??
      ((anchor.signal_type === 'string' || anchor.signal_type === 'string[]') ? anchor.signal_id : null),
    streamUrl: streamFromAnchor,
    poseXSignalId: poseX?.signal_id ?? null,
    poseYSignalId: poseY?.signal_id ?? null,
    headingSignalId: heading?.signal_id ?? null,
    targetsSignalId: targetsSignal?.signal_id ?? null,
    detectionsSignalId: detectionsSignal?.signal_id ?? null,
    showPose: true,
    showTargets: true,
    showDetections: true,
    mirrorX: false,
    sourceWidth: 1280,
    sourceHeight: 720
  };
}

export function readCameraOverlayConfig(
  raw: WidgetConfigRecord | undefined,
  fallbackSignal: SignalRow,
  signals: SignalRow[]
): CameraOverlayConfig {
  const defaults = defaultCameraOverlayConfig(fallbackSignal, signals);
  const base = isObject(raw) ? raw : {};
  const topicPath = toString(base.topicPath, '').trim();

  const streamUrlRaw = toString(base.streamUrl, defaults.streamUrl).trim();
  const streamUrl = streamUrlRaw.length > 0 ? streamUrlRaw : defaults.streamUrl;

  return {
    streamSignalId: resolveConfiguredSignalId(
      base,
      'streamSignalId',
      'streamSignalPath',
      defaults.streamSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    streamUrl,
    poseXSignalId: resolveConfiguredSignalId(
      base,
      'poseXSignalId',
      'poseXSignalPath',
      defaults.poseXSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    poseYSignalId: resolveConfiguredSignalId(
      base,
      'poseYSignalId',
      'poseYSignalPath',
      defaults.poseYSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    headingSignalId: resolveConfiguredSignalId(
      base,
      'headingSignalId',
      'headingSignalPath',
      defaults.headingSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    targetsSignalId: resolveConfiguredSignalId(
      base,
      'targetsSignalId',
      'targetsSignalPath',
      defaults.targetsSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    detectionsSignalId: resolveConfiguredSignalId(
      base,
      'detectionsSignalId',
      'detectionsSignalPath',
      defaults.detectionsSignalId,
      topicPath,
      fallbackSignal,
      signals
    ),
    showPose: toBoolean(base.showPose, defaults.showPose),
    showTargets: toBoolean(base.showTargets, defaults.showTargets),
    showDetections: toBoolean(base.showDetections, defaults.showDetections),
    mirrorX: toBoolean(base.mirrorX, defaults.mirrorX),
    sourceWidth: Math.max(1, Math.floor(toNumber(base.sourceWidth) ?? defaults.sourceWidth)),
    sourceHeight: Math.max(1, Math.floor(toNumber(base.sourceHeight) ?? defaults.sourceHeight))
  };
}

export function normalizedBoxFromUnknown(
  value: unknown,
  sourceWidth: number,
  sourceHeight: number
): CameraOverlayBox | null {
  if (!isObject(value)) return null;

  let x = toNumber(value.x ?? value.left);
  let y = toNumber(value.y ?? value.top);
  let w = toNumber(value.w ?? value.width);
  let h = toNumber(value.h ?? value.height);

  const x2 = toNumber(value.x2 ?? value.right);
  const y2 = toNumber(value.y2 ?? value.bottom);
  const cx = toNumber(value.cx ?? value.centerX);
  const cy = toNumber(value.cy ?? value.centerY);

  if (w === null && x !== null && x2 !== null) {
    w = x2 - x;
  }
  if (h === null && y !== null && y2 !== null) {
    h = y2 - y;
  }

  if (x === null && cx !== null && w !== null) {
    x = cx - w / 2;
  }
  if (y === null && cy !== null && h !== null) {
    y = cy - h / 2;
  }

  if (x === null || y === null || w === null || h === null) return null;
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(w) || !Number.isFinite(h)) {
    return null;
  }
  if (w <= 0 || h <= 0) return null;

  const maxAbs = Math.max(Math.abs(x), Math.abs(y), Math.abs(w), Math.abs(h));
  const pixelSpace = maxAbs > 1.5;

  const nx = pixelSpace ? x / sourceWidth : x;
  const ny = pixelSpace ? y / sourceHeight : y;
  const nw = pixelSpace ? w / sourceWidth : w;
  const nh = pixelSpace ? h / sourceHeight : h;

  const clampedX = clamp(nx, 0, 1);
  const clampedY = clamp(ny, 0, 1);
  const clampedW = clamp(nw, 0.001, 1);
  const clampedH = clamp(nh, 0.001, 1);

  return {
    x: clampedX,
    y: clampedY,
    w: clampedW,
    h: clampedH,
    label: toString(value.label ?? value.class ?? value.name ?? value.id, '').trim(),
    score: toNumber(value.score ?? value.confidence)
  };
}

export function parseOverlayBoxes(
  raw: string,
  sourceWidth: number,
  sourceHeight: number
): CameraOverlayBox[] {
  const trimmed = raw.trim();
  if (!trimmed) return [];

  try {
    const parsed = JSON.parse(trimmed) as unknown;
    const candidates: unknown[] = [];

    if (Array.isArray(parsed)) {
      candidates.push(...parsed);
    } else if (isObject(parsed)) {
      if (Array.isArray(parsed.boxes)) candidates.push(...parsed.boxes);
      if (Array.isArray(parsed.targets)) candidates.push(...parsed.targets);
      if (Array.isArray(parsed.detections)) candidates.push(...parsed.detections);
      if (candidates.length === 0) candidates.push(parsed);
    }

    return candidates
      .map((entry) => normalizedBoxFromUnknown(entry, sourceWidth, sourceHeight))
      .filter((entry): entry is CameraOverlayBox => entry !== null)
      .slice(0, 200);
  } catch {
    return [];
  }
}
