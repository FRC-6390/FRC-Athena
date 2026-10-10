import type { SignalValue } from './types';

/**
 * Injectable command interface (E4/E5). The robot validates every command and answers with a reply.
 * Clients never pre-block: a locked or out-of-range write is sent and the robot's rejection is shown.
 */
export type CommandOp = 'run' | 'cancel' | 'write';

export type CommandReply = {
  op: CommandOp;
  signalId: number;
  accepted: boolean;
  /** Robot-provided reason on rejection, e.g. `rejected: locked (InMatch)`. */
  reason: string | null;
  /** Client wall clock (ms) when the reply arrived. */
  receivedAtMs: number;
};

export interface RobotCommands {
  run(signalId: number): Promise<CommandReply>;
  cancel(signalId: number): Promise<CommandReply>;
  write(signalId: number, value: SignalValue): Promise<CommandReply>;
}
