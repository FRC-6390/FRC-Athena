//! NTP-style clock offset estimate from TimePing/TimePong samples (E4).

/// One round trip: the client's send and receive times and the server's time in between.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeSample {
    pub client_send_micros: u64,
    pub server_micros: u64,
    pub client_recv_micros: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TimeEstimate {
    /// `server clock - client clock` in microseconds. Add it to a client time to get server time.
    pub offset_micros: f64,
    /// Round trip of the sample the offset was taken from (the lowest RTT seen).
    pub rtt_micros: f64,
    /// Number of samples the estimate was taken from.
    pub samples: usize,
}

/// Picks the lowest-RTT sample (the least queued one) and assumes symmetric path delay.
/// Returns `None` when there is no usable sample.
pub fn estimate(samples: &[TimeSample]) -> Option<TimeEstimate> {
    let best = samples
        .iter()
        .filter(|s| s.client_recv_micros >= s.client_send_micros)
        .min_by_key(|s| s.client_recv_micros - s.client_send_micros)?;
    let rtt = (best.client_recv_micros - best.client_send_micros) as f64;
    let midpoint = best.client_send_micros as f64 + rtt / 2.0;
    Some(TimeEstimate {
        offset_micros: best.server_micros as f64 - midpoint,
        rtt_micros: rtt,
        samples: samples.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_lowest_rtt_sample() {
        let samples = [
            TimeSample { client_send_micros: 0, server_micros: 5_000, client_recv_micros: 400 },
            TimeSample { client_send_micros: 1_000, server_micros: 6_010, client_recv_micros: 1_020 },
            TimeSample { client_send_micros: 2_000, server_micros: 7_500, client_recv_micros: 2_300 },
        ];
        let est = estimate(&samples).unwrap();
        assert_eq!(est.rtt_micros, 20.0);
        assert_eq!(est.samples, 3);
        // Midpoint of the best sample is 1010; the server read 6010.
        assert_eq!(est.offset_micros, 5_000.0);
    }

    #[test]
    fn empty_or_broken_samples_give_no_estimate() {
        assert!(estimate(&[]).is_none());
        let backwards = [TimeSample { client_send_micros: 10, server_micros: 1, client_recv_micros: 5 }];
        assert!(estimate(&backwards).is_none());
    }
}
