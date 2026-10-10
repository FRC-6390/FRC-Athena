<script lang="ts">
  import { readDifferentialDriveConfig, readSwerveDriveConfig, readSwerveModuleConfig } from '../../lib/widget-config';
  import type { SignalSectionProps } from './types';

  let {
    selectedWidget,
    primarySignal,
    selectedSignal,
    signals,
    signalById,
    pools,
    setConfig
  }: SignalSectionProps = $props();

  const numericSignals = $derived(pools.numericSignals);
  const boolSignals = $derived(pools.boolSignals);

</script>

{#if selectedWidget.kind === 'swerve_module'}
  {@const swerveModule = readSwerveModuleConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Swerve Module</h4>
    <div class="row-grid">
      <label>
        Label
        <input
          value={swerveModule.label}
          oninput={(event) => setConfig({ ...swerveModule, label: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>

      <label>
        Angle signal
        <select
          value={swerveModule.angleSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, angleSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Auto / none</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Speed signal
        <select
          value={swerveModule.speedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, speedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">Auto / none</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={swerveModule.maxSpeed}
          oninput={(event) =>
            setConfig({ ...swerveModule, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>

      <label>
        Command angle signal
        <select
          value={swerveModule.commandAngleSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, commandAngleSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Command speed signal
        <select
          value={swerveModule.commandSpeedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, commandSpeedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Command offset signal
        <select
          value={swerveModule.commandOffsetSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, commandOffsetSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Command inverted signal
        <select
          value={swerveModule.commandInvertedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveModule, commandInvertedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each boolSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'swerve_drive'}
  {@const swerveDrive = readSwerveDriveConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Swerve Drive</h4>
    <div class="row-grid">
      <label>
        Heading signal
        <select
          value={swerveDrive.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...swerveDrive, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={swerveDrive.maxSpeed}
          oninput={(event) =>
            setConfig({ ...swerveDrive, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>
    </div>

    <div class="series-list">
      {#each swerveDrive.modules as module, index (module.key)}
        <div class="series-row">
          <strong>{module.key.toUpperCase()}</strong>
          <input
            value={module.label}
            oninput={(event) =>
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index
                    ? { ...entry, label: (event.currentTarget as HTMLInputElement).value }
                    : entry
                )
              })}
          />
          <select
            value={module.angleSignalId ?? ''}
            onchange={(event) => {
              const raw = (event.currentTarget as HTMLSelectElement).value;
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index ? { ...entry, angleSignalId: raw ? Number(raw) : null } : entry
                )
              });
            }}
          >
            <option value="">Angle</option>
            {#each numericSignals as option (option.signal_id)}
              <option value={option.signal_id}>{option.path}</option>
            {/each}
          </select>
          <select
            value={module.speedSignalId ?? ''}
            onchange={(event) => {
              const raw = (event.currentTarget as HTMLSelectElement).value;
              setConfig({
                ...swerveDrive,
                modules: swerveDrive.modules.map((entry, moduleIndex) =>
                  moduleIndex === index ? { ...entry, speedSignalId: raw ? Number(raw) : null } : entry
                )
              });
            }}
          >
            <option value="">Speed</option>
            {#each numericSignals as option (option.signal_id)}
              <option value={option.signal_id}>{option.path}</option>
            {/each}
          </select>
        </div>
      {/each}
    </div>
  </section>
{:else if selectedWidget.kind === 'differential_drive'}
  {@const diff = readDifferentialDriveConfig(selectedWidget.config, primarySignal, signals)}
  <section class="config-block">
    <h4>Differential Drive</h4>
    <div class="row-grid">
      <label>
        Left speed signal
        <select
          value={diff.leftSpeedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, leftSpeedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Right speed signal
        <select
          value={diff.rightSpeedSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, rightSpeedSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Heading signal
        <select
          value={diff.headingSignalId ?? ''}
          onchange={(event) => {
            const raw = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...diff, headingSignalId: raw ? Number(raw) : null });
          }}
        >
          <option value="">None</option>
          {#each numericSignals as option (option.signal_id)}
            <option value={option.signal_id}>{option.path}</option>
          {/each}
        </select>
      </label>

      <label>
        Max speed (m/s)
        <input
          type="number"
          min="0.1"
          step="0.1"
          value={diff.maxSpeed}
          oninput={(event) =>
            setConfig({ ...diff, maxSpeed: Math.max(0.1, Number((event.currentTarget as HTMLInputElement).value) || 5) })}
        />
      </label>
    </div>
  </section>
{/if}
