/**
 * Quick-connect targets for the connection bar. The simulator runs on this machine; the robot is
 * reached over USB, radio (10.TE.AM.2) or mDNS (roboRIO-TEAM-frc.local). Team-based hosts need the team
 * number: without it they keep the placeholder and `needsTeam` is true.
 */
import { teamHost } from './live-source';

export type ConnectionTarget = {
  label: string;
  host: string;
  /** True when the host still contains the team placeholder. */
  needsTeam: boolean;
};

export function connectionTargets(team?: number): ConnectionTarget[] {
  const radio = team === undefined ? null : teamHost(team);
  const mdns = team === undefined ? null : team >= 1 && team <= 9999 ? `roborio-${team}-frc.local` : null;
  return [
    { label: 'Simulator', host: '127.0.0.1', needsTeam: false },
    { label: 'Robot (USB)', host: '172.22.11.2', needsTeam: false },
    { label: 'Robot (radio)', host: radio ?? '10.TE.AM.2', needsTeam: radio === null },
    { label: 'Robot (mDNS)', host: mdns ?? 'roborio-TEAM-frc.local', needsTeam: mdns === null }
  ];
}
