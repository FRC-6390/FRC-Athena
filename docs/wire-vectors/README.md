# ARCP v2 golden wire vectors

Each `*.hex` file is one lowercase hex string (a single line, newline terminated) of bytes in the
ARCP v2 / replay format from appendix E1-W of `docs/2027-offseason-design.md`. They are shared by
the two implementations:

- **Java reference**: `athena-runtime` `ca.frc6390.athena.runtime.wire` (`WireWriter`, `WireReader`,
  `Wire`). The Java generator that built these files lived in `athena-arcp` and was removed with the
  pure-Java ARCP server (E7). A format change must update the reference codec, the Rust codec and
  these files together.
- **Rust**: `athena-arcp/rust/crates/arcp-core/tests/golden_vectors.rs` decodes each file and
  re-encodes it byte for byte.

The Rust ARCP server is the only ARCP implementation in `athena-arcp`; it encodes frames and control replies
that the Java tests decode with `athena-runtime`.

| File | Contents |
| --- | --- |
| `dictionary.hex` | One DICTIONARY record with five entries: a writable, persisted `f64` (role 1 state, unit `deg`, lock group `ca.frc6390.athena.runtime.Locks.InMatch`); a `bool` value (role 2); a struct `Pose2d` (role 4 device, schema 1); a `string` hook-free value (role 1); and the lock-state signal `athena/locks/ca.frc6390.athena.runtime.Locks.InMatch` (bool, role 7). The lock group is the fully qualified name so clients can join it with the lock signal. |
| `struct_schema.hex` | One STRUCT_SCHEMA record: id 1, type `Pose2d`, WPILib schema `Translation2d translation;Rotation2d rotation`. |
| `metadata.hex` | One METADATA record with five pairs: event `Practice`, matchType, matchNumber, robot, athenaVersion. |
| `frame_keyframe_all_types.hex` | One keyframe FRAME (flags `0x01`, cycle timestamp 1234567890, sequence 42) with nine entries, one per type tag 1 to 9. Signal ids are tag times 37, so id 333 needs a two-byte varint. The struct entry is schema 1 with 24 bytes (three doubles). |
| `frame_delta.hex` | One non-keyframe FRAME (flags `0x00`, timestamp +20 ms, sequence 43) with two changed entries: an `i64` at id 74 and a `bool` at id 37. |
| `frame_fragment.hex` | One FRAME with the fragment flag (`0x02`), fragment index 1 of 2, sequence 44, and one `string` entry. |
| `marker.hex` | Two MARKER records: a FAULT with a negative timestamp (zigzag, -5000 us, message `motor fault`, no signal) and a user marker (kind 100) tied to signal 111. |
| `compressed_block.hex` | One COMPRESSED_BLOCK record. Its inflated content is the keyframe, delta and marker records above, in that order. The DEFLATE bytes are not canonical, so tests compare the inflated stream, not the compressed bytes. |
| `replay_sample.athrp.hex` | A complete `.athrp` file: `ATHRP\0`, u16 version 1, then METADATA, DICTIONARY and STRUCT_SCHEMA records, then two COMPRESSED_BLOCKs (keyframe, delta, two markers; keyframe, fragment). |

The Rust tests check that the uncompressed prefix of `replay_sample.athrp.hex` is byte identical.
