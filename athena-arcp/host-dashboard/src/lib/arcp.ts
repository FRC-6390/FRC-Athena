import { invoke } from '@tauri-apps/api/core';

export type SignalRow = {
  signal_id: number;
  signal_type: string;
  kind: string;
  access: string;
  policy: string;
  durability: string;
  path: string;
  value: string;
};

export type DashboardSnapshot = {
  connected: boolean;
  status: string;
  signal_count: number;
  update_count: number;
  uptime_ms: number;
  server_cpu_percent: number | null;
  server_rss_bytes: number | null;
  host_cpu_percent: number | null;
  host_rss_bytes: number | null;
  signals: SignalRow[];
};

export type ConnectionInfo = {
  connected: boolean;
  host: string;
  control_port: number;
  udp_port: number;
  signal_count: number;
};

export type WindowModeSnapshot = {
  presentationMode: boolean;
  dockMode: boolean;
};

function isTauriRuntime(): boolean {
  if (typeof window === 'undefined') return false;
  return '__TAURI_INTERNALS__' in window;
}

export async function connectArcp(host: string, controlPort: number): Promise<ConnectionInfo> {
  return invoke('connect_arcp', {
    host,
    controlPort
  });
}

export async function disconnectArcp(): Promise<void> {
  return invoke('disconnect_arcp');
}

export async function snapshotArcp(): Promise<DashboardSnapshot> {
  return invoke('dashboard_snapshot');
}

export async function setSignal(signalId: number, valueRaw: string): Promise<void> {
  return invoke('set_signal_value', {
    signalId,
    valueRaw
  });
}

export async function fireAction(signalId: number): Promise<void> {
  return invoke('trigger_action', {
    signalId
  });
}

export async function saveServerLayout(layoutName: string, layoutJson: string): Promise<void> {
  return invoke('save_server_layout', {
    layoutName,
    layoutJson
  });
}

export async function loadServerLayout(layoutName: string): Promise<string> {
  return invoke('load_server_layout', {
    layoutName
  });
}

export async function listServerLayouts(): Promise<string[]> {
  return invoke('list_server_layouts');
}

export async function deleteServerLayout(layoutName: string): Promise<void> {
  return invoke('delete_server_layout', {
    layoutName
  });
}

export async function windowModeSnapshot(): Promise<WindowModeSnapshot> {
  if (isTauriRuntime()) {
    return invoke('window_mode_snapshot');
  }

  return {
    presentationMode: typeof document !== 'undefined' && !!document.fullscreenElement,
    dockMode: false
  };
}

export async function setPresentationMode(enabled: boolean): Promise<void> {
  if (isTauriRuntime()) {
    return invoke('set_presentation_mode', { enabled });
  }

  if (typeof document === 'undefined') return;
  if (enabled) {
    if (!document.fullscreenElement) {
      if (!document.documentElement.requestFullscreen) {
        throw new Error('fullscreen is not supported on this platform');
      }
      await document.documentElement.requestFullscreen();
    }
    return;
  }

  if (document.fullscreenElement) {
    await document.exitFullscreen();
  }
}

export async function setDriverstationDockMode(enabled: boolean): Promise<void> {
  if (isTauriRuntime()) {
    return invoke('set_driverstation_dock_mode', { enabled });
  }

  if (enabled) {
    throw new Error('Driver Station dock mode is only available in the desktop app');
  }
}
