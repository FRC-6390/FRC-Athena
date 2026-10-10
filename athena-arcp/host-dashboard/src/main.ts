import { mount } from 'svelte';
import App from './App.svelte';
import AppShell from './ui/AppShell.svelte';
import './app.css';
import './ui/theme.css';

const target = document.getElementById('app');
if (!target) {
  throw new Error('Missing #app root element');
}

// The redesigned shell is the default route. `?ui=classic` keeps the classic dashboard reachable while its
// canvas layouts move into the Dashboards section.
const ui = new URLSearchParams(window.location.search).get('ui');

mount(ui === 'classic' ? App : AppShell, { target });
