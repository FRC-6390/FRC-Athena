import type { CommandOp, CommandReply, RobotCommands, SessionCommands } from './commands';
import { ValueStore, type StoreOptions } from './store';
import type { DataSource, SourceSink } from './source';
import type { DecodedRecord, SignalValue } from './types';

export type Session = {
  readonly store: ValueStore;
  /** Commands routed through the recording gateway: every reply is kept in the store. */
  readonly commands: SessionCommands;
  readonly source: DataSource;
  start(): void;
  stop(): void;
};

/**
 * Wires a data source to a store and wraps its commands so every robot reply (accepted or rejected)
 * is recorded for display. Writes are never pre-validated here (E5). The session assigns request ids.
 */
export function createSession(source: DataSource, options: StoreOptions = {}): Session {
  const store = new ValueStore(options);
  let nextRequestId = 1;

  const record = (reply: CommandReply): CommandReply => {
    store.recordReply(reply);
    return reply;
  };
  const send = (
    op: CommandOp,
    signalId: number,
    fallbackValue: SignalValue | null,
    call: (requestId: number) => Promise<CommandReply>
  ): Promise<CommandReply> => {
    const requestId = nextRequestId++;
    return call(requestId).then(record, (error: unknown) =>
      record({
        requestId,
        op,
        signalId,
        accepted: false,
        reason: `transport error: ${error instanceof Error ? error.message : String(error)}`,
        actualValue: fallbackValue,
        receivedAtMs: Date.now()
      })
    );
  };

  const commands: SessionCommands = {
    run: (signalId) => send('run', signalId, null, (id) => source.commands.run(id, signalId)),
    cancel: (signalId) => send('cancel', signalId, null, (id) => source.commands.cancel(id, signalId)),
    write: (signalId: number, value: SignalValue) =>
      send('write', signalId, store.latestSample(signalId)?.value ?? null, (id) =>
        source.commands.write(id, signalId, value)
      )
  };

  const sink: SourceSink = {
    records: (records: DecodedRecord[]) => store.applyAll(records)
  };

  return {
    store,
    commands,
    source,
    start: () => source.start(sink),
    stop: () => source.stop()
  };
}
