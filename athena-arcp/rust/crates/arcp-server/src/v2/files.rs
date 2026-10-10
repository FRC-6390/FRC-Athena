//! File service (E6): list, download and delete named files.
//!
//! A name is either a plain file in the root directory (`name.json`) or a file inside a named
//! category (`category/name.json`). Categories are fixed by the robot when the server starts
//! (replays, persistent store, models, tuning exports, status). Each name part is a plain file
//! name: separators, `.`/`..`, NUL, drive prefixes and symlinks are rejected, so requests cannot
//! leave the configured directories.

use std::fs;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::time::UNIX_EPOCH;

use arcp_core::v2::control::{FileInfo, MAX_FILE_CHUNK};

/// The directories the file service exposes.
#[derive(Debug, Clone, Default)]
pub(crate) struct Roots {
    /// Plain names in this directory. `None` disables plain names.
    pub root: Option<PathBuf>,
    /// `(category, directory)` pairs; names are `category/file`.
    pub categories: Vec<(String, PathBuf)>,
}

impl Roots {
    pub fn enabled(&self) -> bool {
        self.root.is_some() || !self.categories.is_empty()
    }

    /// Resolves a request name to a directory and plain file name.
    fn locate(&self, name: &str) -> Result<(PathBuf, String), &'static str> {
        match name.split_once('/') {
            None => {
                let root = self.root.clone().ok_or("file service not configured")?;
                checked_path(&root, name)?;
                Ok((root, name.to_owned()))
            }
            Some((category, file)) => {
                let dir = self
                    .categories
                    .iter()
                    .find(|(c, _)| c == category)
                    .map(|(_, d)| d.clone())
                    .ok_or("no such category")?;
                checked_path(&dir, file)?;
                Ok((dir, file.to_owned()))
            }
        }
    }

    pub fn list(&self) -> Result<Vec<FileInfo>, &'static str> {
        let mut out = Vec::new();
        if let Some(root) = &self.root {
            out.extend(list(root)?);
        }
        for (category, dir) in &self.categories {
            // A category directory that does not exist yet is empty.
            for mut info in list(dir).unwrap_or_default() {
                info.name = format!("{category}/{}", info.name);
                out.push(info);
            }
        }
        Ok(out)
    }

    pub fn download(&self, name: &str, offset: u64, max_len: u32) -> Result<(bool, Vec<u8>), &'static str> {
        let (dir, file) = self.locate(name)?;
        download(&dir, &file, offset, max_len)
    }

    pub fn delete(&self, name: &str) -> Result<(), &'static str> {
        let (dir, file) = self.locate(name)?;
        delete(&dir, &file)
    }
}

/// Validates a file name and returns its path under `root`.
pub(crate) fn checked_path(root: &Path, name: &str) -> Result<PathBuf, &'static str> {
    if name.is_empty() || name.len() > 255 {
        return Err("invalid file name");
    }
    if name == "." || name == ".." || name.contains(['/', '\\', '\0', ':']) {
        return Err("invalid file name");
    }
    Ok(root.join(name))
}

fn regular_file(path: &Path) -> Result<fs::Metadata, &'static str> {
    let meta = fs::symlink_metadata(path).map_err(|_| "no such file")?;
    if !meta.is_file() {
        return Err("no such file");
    }
    Ok(meta)
}

pub(crate) fn list(root: &Path) -> Result<Vec<FileInfo>, &'static str> {
    let mut out = Vec::new();
    for entry in fs::read_dir(root).map_err(|_| "file root unavailable")? {
        let Ok(entry) = entry else { continue };
        let Ok(meta) = fs::symlink_metadata(entry.path()) else { continue };
        if !meta.is_file() {
            continue;
        }
        let Some(name) = entry.file_name().to_str().map(str::to_owned) else { continue };
        let modified = meta
            .modified()
            .ok()
            .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
            .map_or(0, |d| d.as_secs());
        out.push(FileInfo { name, size: meta.len(), modified_unix_secs: modified });
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

/// Reads up to `max_len` bytes (capped at [`MAX_FILE_CHUNK`]) from `offset`.
/// Returns `(eof, bytes)`.
pub(crate) fn download(root: &Path, name: &str, offset: u64, max_len: u32) -> Result<(bool, Vec<u8>), &'static str> {
    let path = checked_path(root, name)?;
    let meta = regular_file(&path)?;
    let size = meta.len();
    let want = max_len.min(MAX_FILE_CHUNK) as usize;
    let mut file = fs::File::open(&path).map_err(|_| "cannot open file")?;
    file.seek(SeekFrom::Start(offset.min(size))).map_err(|_| "cannot seek")?;
    let mut buf = vec![0u8; want];
    let mut read = 0usize;
    while read < want {
        match file.read(&mut buf[read..]) {
            Ok(0) => break,
            Ok(n) => read += n,
            Err(_) => return Err("read failed"),
        }
    }
    buf.truncate(read);
    let eof = offset.saturating_add(read as u64) >= size;
    Ok((eof, buf))
}

pub(crate) fn delete(root: &Path, name: &str) -> Result<(), &'static str> {
    let path = checked_path(root, name)?;
    regular_file(&path)?;
    fs::remove_file(&path).map_err(|_| "delete failed")
}
