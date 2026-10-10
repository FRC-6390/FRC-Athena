/**
 * Default ARCP ports. The robot's TCP control port is 5808 and its realtime UDP socket is
 * bound to 5809; the dashboard receives realtime frames on UDP 5807. All three are inside the FRC team
 * range 5800-5810. NT4 uses 5810, so ARCP must not.
 */
export const ARCP_CONTROL_PORT = 5808;
export const ARCP_UDP_PORT = 5807;
export const ARCP_ROBOT_REALTIME_PORT = 5809;

/** Subscription made on connect: every signal at the robot loop rate. The server's bandwidth budget lowers the rate if it must. */
export const DEFAULT_SUBSCRIPTION = '**';
export const DEFAULT_SUBSCRIPTION_HZ = 50;
