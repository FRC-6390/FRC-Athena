#[derive(Debug, Clone)]
pub struct CliArgs {
    pub host: String,
    pub control_port: u16,
    pub refresh_ms: u64,
}

#[derive(Debug, Clone)]
pub struct WebArgs {
    pub host: String,
    pub control_port: u16,
    pub http_bind: String,
    pub http_port: u16,
}

pub fn parse_cli_args<I>(args: I) -> Result<CliArgs, String>
where
    I: IntoIterator<Item = String>,
{
    let mut host = String::from("127.0.0.1");
    let mut control_port: u16 = 5805;
    let mut refresh_ms: u64 = 250;

    let raw: Vec<String> = args.into_iter().collect();
    let mut index = 0_usize;
    while index < raw.len() {
        match raw[index].as_str() {
            "--host" => {
                host = read_next(&raw, index, "--host")?;
                index += 2;
            }
            "--control-port" => {
                control_port =
                    parse_u16(&read_next(&raw, index, "--control-port")?, "--control-port")?;
                index += 2;
            }
            "--refresh-ms" => {
                refresh_ms = parse_u64(&read_next(&raw, index, "--refresh-ms")?, "--refresh-ms")?;
                index += 2;
            }
            "--help" | "-h" => {
                return Err(cli_usage());
            }
            unknown => {
                return Err(format!("unknown argument: {unknown}\n{}", cli_usage()));
            }
        }
    }

    if refresh_ms < 50 {
        return Err(String::from("--refresh-ms must be >= 50"));
    }

    Ok(CliArgs {
        host,
        control_port,
        refresh_ms,
    })
}

pub fn parse_web_args<I>(args: I) -> Result<WebArgs, String>
where
    I: IntoIterator<Item = String>,
{
    let mut host = String::from("127.0.0.1");
    let mut control_port: u16 = 5805;
    let mut http_bind = String::from("127.0.0.1");
    let mut http_port: u16 = 5810;

    let raw: Vec<String> = args.into_iter().collect();
    let mut index = 0_usize;
    while index < raw.len() {
        match raw[index].as_str() {
            "--host" => {
                host = read_next(&raw, index, "--host")?;
                index += 2;
            }
            "--control-port" => {
                control_port =
                    parse_u16(&read_next(&raw, index, "--control-port")?, "--control-port")?;
                index += 2;
            }
            "--http-bind" => {
                http_bind = read_next(&raw, index, "--http-bind")?;
                index += 2;
            }
            "--http-port" => {
                http_port = parse_u16(&read_next(&raw, index, "--http-port")?, "--http-port")?;
                index += 2;
            }
            "--help" | "-h" => {
                return Err(web_usage());
            }
            unknown => {
                return Err(format!("unknown argument: {unknown}\n{}", web_usage()));
            }
        }
    }

    Ok(WebArgs {
        host,
        control_port,
        http_bind,
        http_port,
    })
}

fn read_next(args: &[String], index: usize, name: &str) -> Result<String, String> {
    args.get(index + 1)
        .cloned()
        .ok_or_else(|| format!("missing value for {name}"))
}

fn parse_u16(raw: &str, name: &str) -> Result<u16, String> {
    raw.parse::<u16>()
        .map_err(|_| format!("invalid u16 value for {name}: {raw}"))
}

fn parse_u64(raw: &str, name: &str) -> Result<u64, String> {
    raw.parse::<u64>()
        .map_err(|_| format!("invalid u64 value for {name}: {raw}"))
}

fn cli_usage() -> String {
    String::from(
        "Usage: arcp-dashboard-cli [--host <ip-or-hostname>] [--control-port <u16>] [--refresh-ms <u64>]",
    )
}

fn web_usage() -> String {
    String::from(
        "Usage: arcp-dashboard-web [--host <ip-or-hostname>] [--control-port <u16>] [--http-bind <addr>] [--http-port <u16>]",
    )
}
