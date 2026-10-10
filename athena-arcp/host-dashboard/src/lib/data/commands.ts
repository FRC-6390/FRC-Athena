import type { SignalValue } from './types';

/**
 * Injectable command interface (E4/E5). The robot validates every command and answers with a reply.
 * Clients never pre-block: a locked or out-of-range write is sent and the robot's rejection is shown.
 * The caller assigns `requestId`; the robot echoes it in the reply (E4 reliable control channel).
 */
export type CommandOp = 'run' | 'cancel' | 'write';

/**
 * Structured reply `{requestId, accepted, reason, actualValue}` (E1-W "Write replies"). `actualValue` is the
 * value the robot holds after the command, so a client can restore the displayed value on rejection.
 */
export type CommandReply = {
  requestId: number;
  op: CommandOp;
  signalId: number;
  accepted: boolean;
  /** Robot-provided reason on rejection, e.g. `rejected: locked (InMatch)`. Empty string when accepted. */
  reason: string;
  /** Robot value after the command; null when the robot did not report one (actions, transport errors). */
  actualValue: SignalValue | null;
  /** Client wall clock (ms) when the reply arrived. */
  receivedAtMs: number;
};

export interface RobotCommands {
  run(requestId: number, signalId: number): Promise<CommandReply>;
  cancel(requestId: number, signalId: number): Promise<CommandReply>;
  write(requestId: number, signalId: number, value: SignalValue): Promise<CommandReply>;
}

/** What UI code calls: the session assigns request ids and records every reply. */
export interface SessionCommands {
  run(signalId: number): Promise<CommandReply>;
  cancel(signalId: number): Promise<CommandReply>;
  write(signalId: number, value: SignalValue): Promise<CommandReply>;
}
