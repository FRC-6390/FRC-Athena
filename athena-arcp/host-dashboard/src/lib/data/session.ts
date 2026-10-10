import type { CommandOp, CommandReply, RobotCommands } from './commands';
import { ValueStore, type StoreOptions } from './store';
import type { DataSource, SourceSink } from './source';
import type { DecodedRecord, SignalValue } from './types';

export type Session = {
  readonly store: ValueStore;
  /** Commands routed through the recording gateway: every reply is kept in the store. */
  readonly commands: RobotCommands;
  readonly source: DataSource;
  start(): void;
  stop(): void;
};

/**
 * Wires a data source to a store and wraps its commands so every robot reply (accepted or rejected)
 * is recorded for display. Writes are never pre-validated here (E5).
 */
export function createSession(source: DataSource, options: StoreOptions = {}): Session {
  const store = new ValueStore(options);

  const record = (reply: CommandReply): CommandReply => {
    store.recordReply(reply);
    return reply;
  };
  const send = (op: CommandOp, signalId: number, call: () => Promise<CommandReply>) =>
    call().then(record, (error: unknown) =>
      record({
        op,
        signalId,
        accepted: false,
        reason: `transport error: ${error instanceof Error ? error.message : String(error)}`,
        receivedAtMs: Date.now()
      })
    );

  const commands: RobotCommands = {
    run: (signalId) => send('run', signalId, () => source.commands.run(signalId)),
    cancel: (signalId) => send('cancel', signalId, () => source.commands.cancel(signalId)),
    write: (signalId: number, value: SignalValue) =>
      send('write', signalId, () => source.commands.write(signalId, value))
  };

  const sink: SourceSink = {
    records: (records: DecodedRecord[]) => store.applyAll(records),
    lockState: (signalId, state) => store.setLockState(signalId, state)
  };

  return {
    store,
    commands,
    source,
    start: () => source.start(sink),
    stop: () => source.stop()
  };
}
