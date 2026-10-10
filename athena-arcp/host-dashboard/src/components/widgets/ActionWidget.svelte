<script lang="ts">
  import type { SignalRow } from '../../lib/arcp';
  import { isActionSignal } from '../../lib/dashboard';

  let { signal, onSendAction }: { signal: SignalRow; onSendAction: (signalId: number) => void } = $props();
</script>

<button class="btn btn-danger" disabled={!isActionSignal(signal)} onclick={() => onSendAction(signal.signal_id)}>
  Trigger action
</button>
