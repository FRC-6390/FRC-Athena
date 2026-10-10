//! JNI bindings for the ARCP v2 server. Java (`ca.frc6390.athena.arcp.ArcpNative`) only calls
//! these functions; all protocol work (encoding, subscriptions, sockets, file service) happens in
//! `arcp-server`.
//!
//! Threading: the server runs its own threads. Write and action requests from clients are queued
//! here and answered by the robot thread: Java calls `poll` once per cycle, decides, and returns
//! the answers with `reply`. Rust never calls into the JVM, so no thread attachment is needed.

mod bridge;
mod jni_api;
mod marshal;
