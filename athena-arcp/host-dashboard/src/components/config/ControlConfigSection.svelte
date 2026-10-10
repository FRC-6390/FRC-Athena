<script lang="ts">
  import { formatDropdownOptions, parseDropdownOptionsInput, readChoiceConfig, readDropdownConfig, readInputConfig, readTextAreaConfig, readToggleConfig } from '../../lib/widget-config';
  import type { ChoiceWidgetCommit, ChoiceWidgetDirection, HtmlInputType, InputWidgetCommit, TextAreaWidgetCommit, ToggleWidgetStyle } from '../../lib/widget-config';
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

  let dropdownInput = $state('');
  let dropdownInputWidgetId = $state('');
  let choiceInput = $state('');
  let choiceInputWidgetId = $state('');
  $effect(() => {
    if (selectedWidget.kind !== 'dropdown') return;
    if (!primarySignal) return;
    if (dropdownInputWidgetId === selectedWidget.id) return;
    const dropdown = readDropdownConfig(selectedWidget.config, primarySignal);
    dropdownInputWidgetId = selectedWidget.id;
    dropdownInput = formatDropdownOptions(dropdown.options);
  });
  function commitDropdownOptions() {
    if (!primarySignal) return;
    const dropdown = readDropdownConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...dropdown,
      options: parseDropdownOptionsInput(dropdownInput)
    });
  }
  $effect(() => {
    if ((selectedWidget.kind !== 'button_group' && selectedWidget.kind !== 'radio') || !primarySignal) return;
    if (choiceInputWidgetId === selectedWidget.id) return;
    const choice = readChoiceConfig(selectedWidget.config, primarySignal);
    choiceInputWidgetId = selectedWidget.id;
    choiceInput = formatDropdownOptions(choice.options);
  });
  function commitChoiceOptions() {
    if (!primarySignal) return;
    const choice = readChoiceConfig(selectedWidget.config, primarySignal);
    setConfig({
      ...choice,
      options: parseDropdownOptionsInput(choiceInput)
    });
  }
</script>

{#if selectedWidget.kind === 'dropdown'}
  {@const dropdown = readDropdownConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Dropdown</h4>
    <label>
      Mode
      <select
        value={dropdown.commit}
        onchange={(event) =>
          setConfig({ ...dropdown, commit: (event.currentTarget as HTMLSelectElement).value as 'auto' | 'button' })}
      >
        <option value="button">Set button</option>
        <option value="auto">Auto-commit</option>
      </select>
    </label>

    <label>
      Options (label:value, ...)
      <input
        value={dropdownInput}
        oninput={(event) => (dropdownInput = (event.currentTarget as HTMLInputElement).value)}
        onblur={commitDropdownOptions}
      />
    </label>
    <button class="btn" onclick={commitDropdownOptions}>Apply options</button>
  </section>
{:else if selectedWidget.kind === 'toggle'}
  {@const toggle = readToggleConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Toggle</h4>
    <div class="row-grid">
      <label>
        Style
        <select
          value={toggle.style}
          onchange={(event) => {
            const nextStyle: ToggleWidgetStyle =
              (event.currentTarget as HTMLSelectElement).value === 'button' ? 'button' : 'switch';
            setConfig({
              ...toggle,
              style: nextStyle
            });
          }}
        >
          <option value="switch">Switch</option>
          <option value="button">Button</option>
        </select>
      </label>
      <label>
        True label
        <input
          value={toggle.trueLabel}
          oninput={(event) => setConfig({ ...toggle, trueLabel: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        False label
        <input
          value={toggle.falseLabel}
          oninput={(event) => setConfig({ ...toggle, falseLabel: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>
  </section>
{:else if selectedWidget.kind === 'button_group' || selectedWidget.kind === 'radio'}
  {@const choice = readChoiceConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>{selectedWidget.kind === 'radio' ? 'Radio' : 'Buttons'}</h4>
    <div class="row-grid">
      <label>
        Direction
        <select
          value={choice.direction}
          onchange={(event) => {
            const nextDirection: ChoiceWidgetDirection =
              (event.currentTarget as HTMLSelectElement).value === 'horizontal'
                ? 'horizontal'
                : 'vertical';
            setConfig({
              ...choice,
              direction: nextDirection
            });
          }}
        >
          <option value="vertical">Vertical</option>
          <option value="horizontal">Horizontal</option>
        </select>
      </label>
      <label>
        Commit mode
        <select
          value={choice.commit}
          onchange={(event) => {
            const nextCommit: ChoiceWidgetCommit =
              (event.currentTarget as HTMLSelectElement).value === 'button' ? 'button' : 'auto';
            setConfig({
              ...choice,
              commit: nextCommit
            });
          }}
        >
          <option value="auto">Auto</option>
          <option value="button">Apply button</option>
        </select>
      </label>
      {#if choice.commit === 'button'}
        <label>
          Apply button label
          <input
            value={choice.buttonLabel}
            oninput={(event) => setConfig({ ...choice, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
    </div>

    <label>
      Options (label:value, ...)
      <input
        value={choiceInput}
        oninput={(event) => (choiceInput = (event.currentTarget as HTMLInputElement).value)}
        onblur={commitChoiceOptions}
      />
    </label>
    <button class="btn" onclick={commitChoiceOptions}>Apply options</button>
  </section>
{:else if selectedWidget.kind === 'input'}
  {@const inputCfg = readInputConfig(selectedWidget.config, primarySignal)}
  <section class="config-block">
    <h4>Input</h4>
    <div class="row-grid">
      <label>
        Input type
        <select
          value={inputCfg.inputType}
          onchange={(event) => {
            const nextType = (event.currentTarget as HTMLSelectElement).value;
            setConfig({ ...inputCfg, inputType: nextType as HtmlInputType });
          }}
        >
          <option value="text">text</option>
          <option value="search">search</option>
          <option value="password">password</option>
          <option value="email">email</option>
          <option value="url">url</option>
          <option value="tel">tel</option>
          <option value="number">number</option>
          <option value="range">range</option>
          <option value="date">date</option>
          <option value="time">time</option>
          <option value="datetime-local">datetime-local</option>
          <option value="month">month</option>
          <option value="week">week</option>
          <option value="color">color</option>
        </select>
      </label>
      <label>
        Commit mode
        <select
          value={inputCfg.commit}
          onchange={(event) =>
            setConfig({
              ...inputCfg,
              commit: (event.currentTarget as HTMLSelectElement).value as InputWidgetCommit
            })}
        >
          <option value="auto">Auto</option>
          <option value="enter">Enter</option>
          <option value="blur">Blur</option>
          <option value="button">Set button</option>
        </select>
      </label>
      <label>
        Placeholder
        <input
          value={inputCfg.placeholder}
          oninput={(event) => setConfig({ ...inputCfg, placeholder: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      {#if inputCfg.commit === 'button'}
        <label>
          Button label
          <input
            value={inputCfg.buttonLabel}
            oninput={(event) => setConfig({ ...inputCfg, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
      <label>
        Min
        <input
          value={inputCfg.min}
          oninput={(event) => setConfig({ ...inputCfg, min: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Max
        <input
          value={inputCfg.max}
          oninput={(event) => setConfig({ ...inputCfg, max: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Step
        <input
          value={inputCfg.step}
          oninput={(event) => setConfig({ ...inputCfg, step: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Pattern
        <input
          value={inputCfg.pattern}
          oninput={(event) => setConfig({ ...inputCfg, pattern: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Autocomplete
        <input
          value={inputCfg.autocomplete}
          oninput={(event) => setConfig({ ...inputCfg, autocomplete: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={inputCfg.spellcheck}
        onchange={(event) => setConfig({ ...inputCfg, spellcheck: (event.currentTarget as HTMLInputElement).checked })}
      />
      Spellcheck
    </label>
  </section>
{:else if selectedWidget.kind === 'textarea'}
  {@const textArea = readTextAreaConfig(selectedWidget.config)}
  <section class="config-block">
    <h4>Text Area</h4>
    <div class="row-grid">
      <label>
        Placeholder
        <input
          value={textArea.placeholder}
          oninput={(event) => setConfig({ ...textArea, placeholder: (event.currentTarget as HTMLInputElement).value })}
        />
      </label>
      <label>
        Rows
        <input
          type="number"
          min="2"
          max="16"
          value={textArea.rows}
          oninput={(event) =>
            setConfig({
              ...textArea,
              rows: Math.max(2, Math.min(16, Math.floor(Number((event.currentTarget as HTMLInputElement).value) || 4)))
            })}
        />
      </label>
      <label>
        Max length (blank = none)
        <input
          type="number"
          min="1"
          value={textArea.maxLength ?? ''}
          oninput={(event) => {
            const raw = (event.currentTarget as HTMLInputElement).value.trim();
            const value = raw === '' ? null : Math.max(1, Math.floor(Number(raw) || 0));
            setConfig({ ...textArea, maxLength: value });
          }}
        />
      </label>
      <label>
        Commit mode
        <select
          value={textArea.commit}
          onchange={(event) =>
            setConfig({
              ...textArea,
              commit: (event.currentTarget as HTMLSelectElement).value as TextAreaWidgetCommit
            })}
        >
          <option value="auto">Auto</option>
          <option value="ctrl-enter">Ctrl+Enter</option>
          <option value="blur">Blur</option>
          <option value="button">Set button</option>
        </select>
      </label>
      {#if textArea.commit === 'button' || textArea.commit === 'ctrl-enter'}
        <label>
          Button label
          <input
            value={textArea.buttonLabel}
            oninput={(event) => setConfig({ ...textArea, buttonLabel: (event.currentTarget as HTMLInputElement).value })}
          />
        </label>
      {/if}
    </div>

    <label class="row-inline">
      <input
        type="checkbox"
        checked={textArea.spellcheck}
        onchange={(event) => setConfig({ ...textArea, spellcheck: (event.currentTarget as HTMLInputElement).checked })}
      />
      Spellcheck
    </label>
  </section>
{/if}
