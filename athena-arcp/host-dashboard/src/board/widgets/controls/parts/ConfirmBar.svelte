<script lang="ts">
  /** Inline confirmation for a large or out-of-range write. Nothing is sent until the user chooses Send. */
  let {
    text,
    onconfirm,
    oncancel
  }: { text: string; onconfirm: () => void; oncancel: () => void } = $props();
</script>

<div class="confirm" role="alertdialog" aria-label="Confirm change">
  <span class="msg">{text}</span>
  <span class="buttons">
    <button type="button" class="ui-btn" data-variant="primary" data-size="sm" onclick={onconfirm}>Send</button>
    <button type="button" class="ui-btn" data-size="sm" onclick={oncancel}>Cancel</button>
  </span>
</div>

<style>
  .confirm {
    display: flex;
    align-items: center;
    flex-wrap: wrap;
    gap: 6px 8px;
    padding: 6px 8px;
    border: 1px solid var(--warn);
    border-radius: var(--radius);
    background: var(--warn-soft);
    font-size: var(--fs-xs);
    color: var(--text);
  }
  .msg {
    flex: 1;
    min-width: 0;
  }
  .buttons {
    display: inline-flex;
    gap: 6px;
  }
</style>
