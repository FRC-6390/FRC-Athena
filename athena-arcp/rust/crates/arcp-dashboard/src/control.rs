use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::manifest::ManifestItem;

const CONTROL_CONNECT_TIMEOUT: Duration = Duration::from_millis(500);
const CONTROL_READ_TIMEOUT: Duration = Duration::from_millis(500);

pub struct ControlClient {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
}

impl ControlClient {
    pub fn connect(host: &str, control_port: u16) -> io::Result<Self> {
        let mut last_err = None;
        let mut stream = None;
        for addr in (host, control_port).to_socket_addrs()? {
            match TcpStream::connect_timeout(&addr, CONTROL_CONNECT_TIMEOUT) {
                Ok(candidate) => {
                    stream = Some(candidate);
                    break;
                }
                Err(err) => {
                    last_err = Some(err);
                }
            }
        }
        let stream = match stream {
            Some(stream) => stream,
            None => {
                return Err(last_err.unwrap_or_else(|| {
                    io::Error::new(
                        io::ErrorKind::AddrNotAvailable,
                        "no socket addresses resolved for control endpoint",
                    )
                }));
            }
        };
        stream.set_nodelay(true)?;
        stream.set_read_timeout(Some(CONTROL_READ_TIMEOUT))?;

        let mut reader = BufReader::new(stream.try_clone()?);
        let mut welcome = String::new();
        match reader.read_line(&mut welcome) {
            Ok(0) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "control connection closed before welcome banner",
                ));
            }
            Ok(_) => {}
            Err(err)
                if err.kind() == io::ErrorKind::WouldBlock
                    || err.kind() == io::ErrorKind::TimedOut =>
            {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "timed out waiting for control welcome banner (server may be paused or overloaded)",
                ));
            }
            Err(err) => return Err(err),
        }
        if !welcome.starts_with("WELCOME ") {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "control server did not return welcome banner",
            ));
        }

        Ok(Self { stream, reader })
    }

    pub fn ping(&mut self) -> io::Result<()> {
        self.send_and_expect("PING", "PONG")
    }

    pub fn server_stats(&mut self) -> io::Result<(Option<f32>, Option<u64>)> {
        self.write_line("STATS")?;
        let response = self.read_line_trimmed()?;
        if response == "ERR UNKNOWN" {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "server does not support STATS",
            ));
        }
        let stats = response
            .strip_prefix("STATS ")
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "invalid STATS response"))?;

        let mut cpu_percent = None;
        let mut rss_bytes = None;
        for token in stats.split_whitespace() {
            if let Some(raw) = token.strip_prefix("cpu=") {
                cpu_percent = parse_optional_f32(raw).ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidData, "invalid STATS cpu value")
                })?;
                continue;
            }
            if let Some(raw) = token.strip_prefix("rss=") {
                rss_bytes = parse_optional_u64(raw).ok_or_else(|| {
                    io::Error::new(io::ErrorKind::InvalidData, "invalid STATS rss value")
                })?;
            }
        }

        Ok((cpu_percent, rss_bytes))
    }

    pub fn subscribe(&mut self, udp_port: u16) -> io::Result<()> {
        self.send_and_expect(&format!("SUB {udp_port}"), "OK SUB")
    }

    pub fn unsubscribe(&mut self, udp_port: u16) -> io::Result<()> {
        self.send_and_expect(&format!("UNSUB {udp_port}"), "OK UNSUB")
    }

    pub fn set_value(&mut self, signal_id: u16, value_raw: &str) -> io::Result<()> {
        let mut sanitized = value_raw.replace('\n', " ");
        sanitized = sanitized.replace('\r', " ");
        self.send_and_expect(&format!("SET {signal_id} {sanitized}"), "OK SET")
    }

    pub fn action(&mut self, signal_id: u16) -> io::Result<()> {
        self.send_and_expect(&format!("ACTION {signal_id}"), "OK ACTION")
    }

    pub fn save_layout(&mut self, name: &str, layout_json: &str) -> io::Result<()> {
        validate_layout_name(name)?;
        if layout_json.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "layout payload is empty",
            ));
        }
        let encoded = hex_encode(layout_json.as_bytes());
        self.send_and_expect(&format!("LAYOUT_SAVE {name} {encoded}"), "OK LAYOUT_SAVE")
    }

    pub fn load_layout(&mut self, name: &str) -> io::Result<String> {
        validate_layout_name(name)?;
        self.write_line(&format!("LAYOUT_LOAD {name}"))?;
        let response = self.read_line_trimmed()?;
        if response == "ERR UNKNOWN" {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "server does not support LAYOUT_LOAD",
            ));
        }
        if response == "ERR LAYOUT_LOAD" {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("layout '{name}' not found"),
            ));
        }

        let mut parts = response.splitn(3, ' ');
        let tag = parts.next().unwrap_or_default();
        if tag != "LAYOUT_DATA" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unexpected LAYOUT_LOAD response: {response}"),
            ));
        }

        let loaded_name = parts.next().unwrap_or_default();
        if loaded_name != name {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("loaded layout name mismatch: expected '{name}', got '{loaded_name}'"),
            ));
        }
        let encoded = parts.next().unwrap_or_default();
        let bytes = hex_decode(encoded).map_err(|msg| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid layout payload encoding: {msg}"),
            )
        })?;
        String::from_utf8(bytes).map_err(|err| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("layout payload is not utf8: {err}"),
            )
        })
    }

    pub fn list_layouts(&mut self) -> io::Result<Vec<String>> {
        self.write_line("LAYOUT_LIST")?;
        let response = self.read_line_trimmed()?;
        if response == "ERR UNKNOWN" || response == "ERR LAYOUT_LIST" {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "server does not support LAYOUT_LIST",
            ));
        }

        let mut parts = response.split_whitespace();
        let tag = parts.next().unwrap_or_default();
        if tag != "LAYOUT_LIST" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unexpected LAYOUT_LIST response: {response}"),
            ));
        }
        let count_raw = parts.next().unwrap_or_default();
        let count = count_raw.parse::<usize>().map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid layout count: {count_raw}"),
            )
        })?;
        let names = parts.map(|value| value.to_string()).collect::<Vec<_>>();
        if names.len() != count {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "layout count mismatch: declared {count}, received {}",
                    names.len()
                ),
            ));
        }
        Ok(names)
    }

    pub fn delete_layout(&mut self, name: &str) -> io::Result<()> {
        validate_layout_name(name)?;
        self.write_line(&format!("LAYOUT_DELETE {name}"))?;
        let response = self.read_line_trimmed()?;
        if response == "OK LAYOUT_DELETE" {
            return Ok(());
        }
        if response == "ERR LAYOUT_DELETE" {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("layout '{name}' not found"),
            ));
        }
        if response == "ERR UNKNOWN" {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "server does not support LAYOUT_DELETE",
            ));
        }
        Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unexpected LAYOUT_DELETE response: {response}"),
        ))
    }

    pub fn manifest(&mut self) -> io::Result<Vec<ManifestItem>> {
        self.write_line("MANIFEST")?;
        let begin = self.read_line_trimmed()?;
        let mut parts = begin.split_whitespace();
        let tag = parts.next().unwrap_or_default();
        if tag != "MANIFEST_BEGIN" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unexpected manifest begin line: {begin}"),
            ));
        }
        let count_raw = parts.next().unwrap_or_default();
        let count = count_raw.parse::<usize>().map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidData,
                format!("invalid manifest count: {count_raw}"),
            )
        })?;

        let mut items = Vec::with_capacity(count);
        for _ in 0..count {
            let line = self.read_line_trimmed()?;
            let item = ManifestItem::parse_line(&line).map_err(|msg| {
                io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("failed to parse manifest item '{line}': {msg}"),
                )
            })?;
            items.push(item);
        }

        let end = self.read_line_trimmed()?;
        if end != "MANIFEST_END" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unexpected manifest terminator: {end}"),
            ));
        }

        items.sort_by_key(|item| item.signal_id);
        Ok(items)
    }

    fn send_and_expect(&mut self, command: &str, expected: &str) -> io::Result<()> {
        self.write_line(command)?;
        let response = self.read_line_trimmed()?;
        if response == expected {
            return Ok(());
        }
        Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("command '{command}' failed with response '{response}'"),
        ))
    }

    fn write_line(&mut self, line: &str) -> io::Result<()> {
        self.stream.write_all(line.as_bytes())?;
        self.stream.write_all(b"\n")?;
        self.stream.flush()
    }

    fn read_line_trimmed(&mut self) -> io::Result<String> {
        let mut line = String::new();
        let read = match self.reader.read_line(&mut line) {
            Ok(read) => read,
            Err(err)
                if err.kind() == io::ErrorKind::WouldBlock
                    || err.kind() == io::ErrorKind::TimedOut =>
            {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    "timed out waiting for control response (server may be paused or overloaded)",
                ));
            }
            Err(err) => return Err(err),
        };
        if read == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "control connection closed",
            ));
        }
        Ok(line.trim().to_string())
    }
}

fn validate_layout_name(name: &str) -> io::Result<()> {
    if name.is_empty() || name.len() > 64 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "layout name must be 1..64 characters",
        ));
    }
    if name
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_' | b'.'))
    {
        return Ok(());
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidInput,
        "layout name can only contain [a-zA-Z0-9._-]",
    ))
}

fn hex_encode(raw: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(raw.len().saturating_mul(2));
    for byte in raw {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn hex_decode(raw: &str) -> Result<Vec<u8>, &'static str> {
    if raw.len() % 2 != 0 {
        return Err("hex payload length must be even");
    }
    let mut out = Vec::with_capacity(raw.len() / 2);
    let bytes = raw.as_bytes();
    let mut index = 0_usize;
    while index < bytes.len() {
        let hi = decode_hex_nibble(bytes[index])?;
        let lo = decode_hex_nibble(bytes[index + 1])?;
        out.push((hi << 4) | lo);
        index += 2;
    }
    Ok(out)
}

fn decode_hex_nibble(byte: u8) -> Result<u8, &'static str> {
    match byte {
        b'0'..=b'9' => Ok(byte - b'0'),
        b'a'..=b'f' => Ok(byte - b'a' + 10),
        b'A'..=b'F' => Ok(byte - b'A' + 10),
        _ => Err("invalid hex nibble"),
    }
}

fn parse_optional_f32(raw: &str) -> Option<Option<f32>> {
    if matches!(raw, "na" | "null" | "-") {
        return Some(None);
    }
    raw.parse::<f32>().ok().map(Some)
}

fn parse_optional_u64(raw: &str) -> Option<Option<u64>> {
    if matches!(raw, "na" | "null" | "-") {
        return Some(None);
    }
    raw.parse::<u64>().ok().map(Some)
}
