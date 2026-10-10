// Drives the headless sim's Driver Station over WPILib's WebSocket sim server.
// Usage: bun scripts/sim-ds.mjs <teleop|auto|test|disable> [red|blue]
const [mode = 'teleop', alliance = 'blue'] = process.argv.slice(2);
const port = process.env.HALSIMWS_PORT ?? '3310';
const ws = new WebSocket(`ws://127.0.0.1:${port}/wpilibws`);
const ds = (data) => ws.send(JSON.stringify({ type: 'DriverStation', device: '', data: { ...data, '>new_data': true } }));
ws.onopen = () => {
  const enabled = mode !== 'disable';
  ds({
    '>station': `${alliance}1`,
    '>autonomous': mode === 'auto',
    '>test': mode === 'test',
    '>ds': true,
    '>enabled': enabled
  });
  console.log(enabled ? `enabled ${mode} (${alliance})` : 'disabled');
  setTimeout(() => ws.close(), 200);
};
ws.onerror = () => {
  console.error(`no sim WebSocket server on :${port} (start the sim with -PathenaHeadless)`);
  process.exit(1);
};
