//! Decodes every golden vector in `docs/wire-vectors/` and re-encodes it byte for byte.
//!
//! COMPRESSED_BLOCK payloads are DEFLATE streams whose exact bytes depend on the encoder, so for
//! those the test checks the inflated inner record stream byte for byte and checks that a block
//! produced by this crate decodes back to the same records.

use std::fs;
use std::path::PathBuf;

use arcp_core::v2::record::{
    decode_compressed_payload, decode_replay, decode_stream, encode_compressed_block, encode_stream,
    replay_header, Record,
};
use arcp_core::v2::{Entry, Frame, Value};

fn vector_bytes(name: &str) -> Vec<u8> {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../../docs/wire-vectors").join(name);
    let text = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let hex = text.trim();
    assert_eq!(hex.len() % 2, 0, "{name}: odd hex length");
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("hex digit"))
        .collect()
}

/// Plain (uncompressed) vectors: decode, then re-encode and compare bytes.
#[test]
fn plain_vectors_round_trip_byte_identically() {
    for name in [
        "dictionary.hex",
        "struct_schema.hex",
        "metadata.hex",
        "frame_keyframe_all_types.hex",
        "frame_delta.hex",
        "frame_fragment.hex",
        "marker.hex",
    ] {
        let bytes = vector_bytes(name);
        let records = decode_stream(&bytes).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(!records.is_empty(), "{name}");
        assert!(
            records.iter().all(|r| !matches!(r, Record::Unknown { .. })),
            "{name}: every record type should be known"
        );
        assert_eq!(encode_stream(&records), bytes, "{name} re-encode mismatch");
    }
}

#[test]
fn keyframe_contains_every_type_tag() {
    let records = decode_stream(&vector_bytes("frame_keyframe_all_types.hex")).unwrap();
    let Record::Frame(frame) = &records[0] else { panic!("expected frame") };
    assert!(frame.keyframe());
    assert!(!frame.fragmented());
    assert_eq!(frame.timestamp_micros, 1_234_567_890);
    assert_eq!(frame.sequence, 42);
    let mut tags: Vec<u8> = frame.entries.iter().map(|e| e.value.type_tag()).collect();
    tags.sort_unstable();
    assert_eq!(tags, (1..=9).collect::<Vec<u8>>());
    // Signal ids are tag * 37, so tag 9 (struct) is id 333 (two-byte varint).
    assert!(frame.entries.iter().any(|e| e.signal_id == 333
        && matches!(&e.value, Value::Struct { schema_id: 1, bytes } if bytes.len() == 24)));
    assert!(frame.entries.iter().any(|e| matches!(&e.value, Value::Str(s) if s == "hello \u{2713}")));
}

#[test]
fn delta_and_fragment_flags_decode() {
    let delta = decode_stream(&vector_bytes("frame_delta.hex")).unwrap();
    let Record::Frame(delta) = &delta[0] else { panic!("expected frame") };
    assert!(!delta.keyframe());
    assert_eq!(delta.entries.len(), 2);

    let fragment = decode_stream(&vector_bytes("frame_fragment.hex")).unwrap();
    let Record::Frame(fragment) = &fragment[0] else { panic!("expected frame") };
    assert!(fragment.fragmented());
    assert_eq!((fragment.fragment_index, fragment.fragment_count), (1, 2));
}

#[test]
fn compressed_block_inner_stream_is_byte_identical() {
    let bytes = vector_bytes("compressed_block.hex");
    let records = decode_stream(&bytes).unwrap();
    let mut expected = vector_bytes("frame_keyframe_all_types.hex");
    expected.extend(vector_bytes("frame_delta.hex"));
    expected.extend(vector_bytes("marker.hex"));
    assert_eq!(records.len(), 4, "keyframe, delta and two markers");
    assert_eq!(encode_stream(&records), expected);

    // Our own encoder produces a block that decodes to the same records.
    let rebuilt = encode_compressed_block(&records);
    assert_eq!(decode_stream(&rebuilt).unwrap(), records);
}

#[test]
fn compressed_payload_rejects_wrong_declared_length() {
    let bytes = vector_bytes("compressed_block.hex");
    // Record envelope: type byte, varint length, then payload (varint uncompressedLength, deflate).
    let payload = &bytes[2..];
    let mut tampered = payload.to_vec();
    tampered[0] = tampered[0].wrapping_add(1);
    assert!(decode_compressed_payload(&tampered).is_err());
}

#[test]
fn replay_sample_decodes_and_plain_prefix_is_byte_identical() {
    let file = vector_bytes("replay_sample.athrp.hex");
    let records = decode_replay(&file).unwrap();
    assert_eq!(records.len(), 3 + 4 + 2, "three header records, then blocks of 4 and 2 records");

    // Header (magic, version) followed by the three plain records is byte-identical.
    let mut prefix = replay_header();
    prefix.extend(encode_stream(&records[..3]));
    assert_eq!(&file[..prefix.len()], prefix.as_slice());

    // Rebuilding the file with this crate's deflate yields the same records.
    let mut rebuilt = replay_header();
    rebuilt.extend(encode_stream(&records[..3]));
    rebuilt.extend(encode_compressed_block(&records[3..7]));
    rebuilt.extend(encode_compressed_block(&records[7..]));
    assert_eq!(decode_replay(&rebuilt).unwrap(), records);
}

#[test]
fn every_vector_is_present_and_decodable_as_a_single_frame_stream() {
    // Sanity check that frames built in Rust match the Java-encoded delta vector layout.
    let frame = Frame::new(
        false,
        1_234_587_890,
        43,
        vec![
            Entry { signal_id: 74, value: Value::I64(-1234567889999) },
            Entry { signal_id: 37, value: Value::Bool(false) },
        ],
    );
    let encoded = encode_stream(&[Record::Frame(frame)]);
    assert_eq!(encoded, vector_bytes("frame_delta.hex"));
}
