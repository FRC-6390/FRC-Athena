import type { RobotCommands } from './commands';
import type { DecodedRecord, LockState } from './types';

/** Where a data source delivers what it receives. */
export interface SourceSink {
  records(records: DecodedRecord[]): void;
  lockState(signalId: number, state: LockState): void;
}

/** A transport or simulator that feeds the store and accepts commands. */
export interface DataSource {
  readonly name: string;
  readonly commands: RobotCommands;
  start(sink: SourceSink): void;
  stop(): void;
}
