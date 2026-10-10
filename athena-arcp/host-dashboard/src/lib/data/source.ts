import type { RobotCommands } from './commands';
import type { DecodedRecord } from './types';

/**
 * Where a data source delivers what it receives. Lock state is not pushed: it is a role-7 signal
 * on the wire and the store derives it (`ValueStore.lockStateOf`).
 */
export interface SourceSink {
  records(records: DecodedRecord[]): void;
}

/** A transport or simulator that feeds the store and accepts commands. */
export interface DataSource {
  readonly name: string;
  readonly commands: RobotCommands;
  start(sink: SourceSink): void;
  stop(): void;
}
