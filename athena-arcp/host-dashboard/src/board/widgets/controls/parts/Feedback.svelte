<script lang="ts">
  import type { CommandReply } from '../../../../lib/data/commands';

  /** Shared write feedback: in-flight, lock hint (informational only), and the robot's reply. */
  let {
    reply,
    locked = false,
    pending = false,
    now = 0
  }: { reply?: CommandReply; locked?: boolean; pending?: boolean; now?: number } = $props();

  const fresh = $derived(!!reply && reply.accepted && now - reply.receivedAtMs < 4000);
</script>

<span class="feedback">
  {#if pending}<span class="pending">sending</span>{/if}
  {#if locked}<span class="locked" title="The robot reports this signal's lock group as locked. The write is still sent.">locked</span>{/if}
  {#if reply && !reply.accepted}<span class="rejected" title={reply.reason}>rejected{reply.reason ? `: ${reply.reason}` : ''}</span>{/if}
  {#if fresh && !pending}<span class="accepted">accepted</span>{/if}
</span>

<style>
  .feedback {
    display: inline-flex;
    flex-wrap: wrap;
    gap: 6px;
    min-width: 0;
    font-size: var(--fs-xs);
  }
  .pending {
    color: var(--text-soft);
  }
  .locked {
    color: var(--locked);
    font-weight: 600;
  }
  .rejected {
    color: var(--fault);
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    max-width: 100%;
  }
  .accepted {
    color: var(--ok);
  }
</style>
