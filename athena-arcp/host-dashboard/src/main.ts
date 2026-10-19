import { mount } from 'svelte';
import AppShell from './ui/AppShell.svelte';
import './app.css';
import './ui/theme.css';

const target = document.getElementById('app');
if (!target) {
  throw new Error('Missing #app root element');
}

mount(AppShell, { target });
