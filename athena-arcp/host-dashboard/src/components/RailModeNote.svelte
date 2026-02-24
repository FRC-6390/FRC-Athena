<script lang="ts">
  type Props = {
    label: string;
    hint: string;
  };

  let { label, hint }: Props = $props();
</script>

<section class="rail-mode-note">
  <strong>{label}</strong>
  <span>{hint}</span>
</section>

<style>
  .rail-mode-note {
    border: 1px solid var(--border-subtle);
    border-radius: 8px;
    background: var(--surface-2);
    padding: 0.34rem 0.54rem;
    display: flex;
    align-items: center;
    gap: 0.5rem;
    min-height: 2rem;
  }

  .rail-mode-note strong {
    font-size: 0.74rem;
    color: var(--text-strong);
    font-family: var(--font-display);
    letter-spacing: 0.01em;
  }

  .rail-mode-note span {
    font-size: 0.72rem;
    color: var(--text-soft);
  }
</style>
