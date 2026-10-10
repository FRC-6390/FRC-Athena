import { mount } from 'svelte';
import App from './App.svelte';
import NextShell from './NextShell.svelte';
import './app.css';

const target = document.getElementById('app');
if (!target) {
  throw new Error('Missing #app root element');
}

// The workspace shell (driver/pit/tuning/analysis) is the default route. `?ui=classic` keeps the
// classic dashboard reachable until the workspaces cover it.
const ui = new URLSearchParams(window.location.search).get('ui');

mount(ui === 'classic' ? App : NextShell, { target });
