use std::sync::atomic::{AtomicBool, Ordering};

use serde::Serialize;
use tauri::{PhysicalPosition, PhysicalSize, State, Window};
#[cfg(target_os = "windows")]
use windows_sys::Win32::Foundation::{BOOL, HWND, LPARAM, RECT};
#[cfg(target_os = "windows")]
use windows_sys::Win32::Graphics::Gdi::{
    GetMonitorInfoW, MonitorFromWindow, MONITORINFO, MONITOR_DEFAULTTONEAREST,
};
#[cfg(target_os = "windows")]
use windows_sys::Win32::UI::WindowsAndMessaging::{
    EnumWindows, GetWindowRect, GetWindowTextLengthW, GetWindowTextW, IsIconic, IsWindowVisible,
};

mod live;

#[derive(Default)]
struct AppRuntime {
    presentation_mode: AtomicBool,
    dock_mode: AtomicBool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct WindowModeSnapshot {
    presentation_mode: bool,
    dock_mode: bool,
}

#[tauri::command]
fn window_mode_snapshot(
    runtime: State<'_, AppRuntime>,
    window: Window,
) -> Result<WindowModeSnapshot, String> {
    let presentation_mode = window
        .is_fullscreen()
        .map_err(|err| format!("read fullscreen state failed: {err}"))?;
    runtime
        .presentation_mode
        .store(presentation_mode, Ordering::SeqCst);

    Ok(WindowModeSnapshot {
        presentation_mode,
        dock_mode: runtime.dock_mode.load(Ordering::SeqCst),
    })
}

#[tauri::command]
fn set_presentation_mode(
    runtime: State<'_, AppRuntime>,
    window: Window,
    enabled: bool,
) -> Result<(), String> {
    window
        .set_fullscreen(enabled)
        .map_err(|err| format!("set fullscreen failed: {err}"))?;

    if enabled {
        runtime.dock_mode.store(false, Ordering::SeqCst);
        window
            .set_always_on_top(false)
            .map_err(|err| format!("disable always-on-top failed: {err}"))?;
    } else if !runtime.dock_mode.load(Ordering::SeqCst) {
        window
            .set_decorations(true)
            .map_err(|err| format!("restore window decorations failed: {err}"))?;
        window
            .set_resizable(true)
            .map_err(|err| format!("restore window resize failed: {err}"))?;
    }

    runtime.presentation_mode.store(enabled, Ordering::SeqCst);
    if enabled {
        let _ = window.set_focus();
    }
    Ok(())
}

fn default_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
    let monitor = match window
        .current_monitor()
        .map_err(|err| format!("read current monitor failed: {err}"))?
    {
        Some(monitor) => monitor,
        None => window
            .primary_monitor()
            .map_err(|err| format!("read primary monitor failed: {err}"))?
            .ok_or_else(|| String::from("no monitor available"))?,
    };

    let work_area = monitor.work_area();
    let width = work_area.size.width;
    let max_height = work_area.size.height;
    let min_height = max_height.min(260);
    let suggested_height = ((max_height as f64) * 0.37).round() as u32;
    let height = suggested_height.clamp(min_height, max_height);

    Ok((
        PhysicalPosition::new(work_area.position.x, work_area.position.y),
        PhysicalSize::new(width, height),
    ))
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy)]
struct DriverStationWindowMatch {
    hwnd: HWND,
    rect: RECT,
    area: i64,
}

#[cfg(target_os = "windows")]
#[derive(Clone, Copy)]
struct WindowRect {
    left: i32,
    top: i32,
    right: i32,
    bottom: i32,
}

#[cfg(target_os = "windows")]
impl WindowRect {
    fn from_rect(rect: RECT) -> Self {
        Self {
            left: rect.left,
            top: rect.top,
            right: rect.right,
            bottom: rect.bottom,
        }
    }

    fn width(self) -> i32 {
        (self.right - self.left).max(0)
    }

    fn height(self) -> i32 {
        (self.bottom - self.top).max(0)
    }

    fn area(self) -> i64 {
        (self.width() as i64).saturating_mul(self.height() as i64)
    }
}

#[cfg(target_os = "windows")]
fn normalize_window_title(raw: &str) -> String {
    raw.chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}

#[cfg(target_os = "windows")]
fn find_driverstation_window() -> Option<(HWND, RECT)> {
    unsafe extern "system" fn enum_windows_proc(hwnd: HWND, lparam: LPARAM) -> BOOL {
        if lparam == 0 {
            return 1;
        }
        if unsafe { IsWindowVisible(hwnd) } == 0 || unsafe { IsIconic(hwnd) } != 0 {
            return 1;
        }

        let title_len = unsafe { GetWindowTextLengthW(hwnd) };
        if title_len <= 0 {
            return 1;
        }

        let mut title_buf = vec![0_u16; (title_len as usize) + 1];
        let written =
            unsafe { GetWindowTextW(hwnd, title_buf.as_mut_ptr(), title_buf.len() as i32) };
        if written <= 0 {
            return 1;
        }

        let title = String::from_utf16_lossy(&title_buf[..written as usize]);
        let normalized = normalize_window_title(&title);
        if !normalized.contains("driverstation") {
            return 1;
        }

        let mut rect: RECT = unsafe { std::mem::zeroed() };
        if unsafe { GetWindowRect(hwnd, &mut rect) } == 0 {
            return 1;
        }
        let width = (rect.right - rect.left).max(0) as i64;
        let height = (rect.bottom - rect.top).max(0) as i64;
        let area = width.saturating_mul(height);
        if area == 0 {
            return 1;
        }

        let best_match = unsafe { &mut *(lparam as *mut Option<DriverStationWindowMatch>) };
        let replace = best_match
            .as_ref()
            .map(|current| area > current.area)
            .unwrap_or(true);
        if replace {
            *best_match = Some(DriverStationWindowMatch { hwnd, rect, area });
        }

        1
    }

    let mut best_match: Option<DriverStationWindowMatch> = None;
    unsafe {
        EnumWindows(
            Some(enum_windows_proc),
            (&mut best_match as *mut Option<DriverStationWindowMatch>) as LPARAM,
        );
    }
    best_match.map(|entry| (entry.hwnd, entry.rect))
}

#[cfg(target_os = "windows")]
fn monitor_work_area_for_window(hwnd: HWND) -> Option<RECT> {
    let monitor = unsafe { MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST) };
    if monitor.is_null() {
        return None;
    }

    let mut info: MONITORINFO = unsafe { std::mem::zeroed() };
    info.cbSize = std::mem::size_of::<MONITORINFO>() as u32;
    let ok = unsafe { GetMonitorInfoW(monitor, &mut info as *mut MONITORINFO as *mut _) };
    if ok == 0 {
        return None;
    }
    Some(info.rcWork)
}

#[cfg(target_os = "windows")]
fn clamp_rect(rect: WindowRect, bounds: WindowRect) -> WindowRect {
    let mut left = rect.left.clamp(bounds.left, bounds.right);
    let mut right = rect.right.clamp(bounds.left, bounds.right);
    if right < left {
        std::mem::swap(&mut left, &mut right);
    }

    let mut top = rect.top.clamp(bounds.top, bounds.bottom);
    let mut bottom = rect.bottom.clamp(bounds.top, bounds.bottom);
    if bottom < top {
        std::mem::swap(&mut top, &mut bottom);
    }

    WindowRect {
        left,
        top,
        right,
        bottom,
    }
}

#[cfg(target_os = "windows")]
fn resolve_windows_driverstation_dock_geometry(
) -> Option<(PhysicalPosition<i32>, PhysicalSize<u32>)> {
    let (driverstation_hwnd, driverstation_rect_raw) = find_driverstation_window()?;
    let work_area_raw = monitor_work_area_for_window(driverstation_hwnd)?;

    let work_area = WindowRect::from_rect(work_area_raw);
    if work_area.width() <= 0 || work_area.height() <= 0 {
        return None;
    }

    let driverstation_rect = clamp_rect(WindowRect::from_rect(driverstation_rect_raw), work_area);
    if driverstation_rect.width() <= 0 || driverstation_rect.height() <= 0 {
        return None;
    }

    let candidates = [
        WindowRect {
            left: work_area.left,
            top: work_area.top,
            right: work_area.right,
            bottom: driverstation_rect.top,
        },
        WindowRect {
            left: work_area.left,
            top: driverstation_rect.bottom,
            right: work_area.right,
            bottom: work_area.bottom,
        },
        WindowRect {
            left: work_area.left,
            top: work_area.top,
            right: driverstation_rect.left,
            bottom: work_area.bottom,
        },
        WindowRect {
            left: driverstation_rect.right,
            top: work_area.top,
            right: work_area.right,
            bottom: work_area.bottom,
        },
    ];

    let best = candidates
        .iter()
        .copied()
        .filter(|rect| rect.width() > 0 && rect.height() > 0)
        .max_by_key(|rect| rect.area())?;

    if best.width() < 320 || best.height() < 180 {
        return None;
    }

    Some((
        PhysicalPosition::new(best.left, best.top),
        PhysicalSize::new(best.width() as u32, best.height() as u32),
    ))
}

#[cfg(target_os = "windows")]
fn resolve_driverstation_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
    if let Some(geometry) = resolve_windows_driverstation_dock_geometry() {
        return Ok(geometry);
    }
    default_dock_geometry(window)
}

#[cfg(not(target_os = "windows"))]
fn resolve_driverstation_dock_geometry(
    window: &Window,
) -> Result<(PhysicalPosition<i32>, PhysicalSize<u32>), String> {
    default_dock_geometry(window)
}

#[tauri::command]
fn set_driverstation_dock_mode(
    runtime: State<'_, AppRuntime>,
    window: Window,
    enabled: bool,
) -> Result<(), String> {
    if !enabled {
        runtime.dock_mode.store(false, Ordering::SeqCst);
        window
            .set_always_on_top(false)
            .map_err(|err| format!("disable always-on-top failed: {err}"))?;
        window
            .set_decorations(true)
            .map_err(|err| format!("restore window decorations failed: {err}"))?;
        window
            .set_resizable(true)
            .map_err(|err| format!("restore window resize failed: {err}"))?;
        return Ok(());
    }

    window
        .set_fullscreen(false)
        .map_err(|err| format!("exit fullscreen failed: {err}"))?;
    runtime.presentation_mode.store(false, Ordering::SeqCst);

    let (dock_position, dock_size) = resolve_driverstation_dock_geometry(&window)?;

    window
        .set_decorations(false)
        .map_err(|err| format!("set borderless mode failed: {err}"))?;
    window
        .set_resizable(false)
        .map_err(|err| format!("set non-resizable failed: {err}"))?;
    window
        .set_size(dock_size)
        .map_err(|err| format!("set dock size failed: {err}"))?;
    window
        .set_position(dock_position)
        .map_err(|err| format!("set dock position failed: {err}"))?;
    window
        .set_always_on_top(true)
        .map_err(|err| format!("enable always-on-top failed: {err}"))?;

    let _ = window.set_focus();
    runtime.dock_mode.store(true, Ordering::SeqCst);
    Ok(())
}

pub fn run() {
    tauri::Builder::default()
        .manage(AppRuntime::default())
        .manage(live::LiveRuntime::default())
        .invoke_handler(tauri::generate_handler![
            window_mode_snapshot,
            set_presentation_mode,
            set_driverstation_dock_mode,
            live::live_connect,
            live::live_disconnect,
            live::live_status,
            live::live_subscribe,
            live::live_unsubscribe,
            live::live_write,
            live::live_action,
            live::live_time_sync,
            live::live_schema_request,
            live::live_file_list,
            live::live_file_download,
            live::live_file_delete,
            live::live_pull_tuning,
            live::live_tuning_diff,
            live::live_record_start,
            live::live_record_stop,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
