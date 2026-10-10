import { mount } from 'svelte';
import App from './App.svelte';
import NextShell from './NextShell.svelte';
import './app.css';

const target = document.getElementById('app');
if (!target) {
  throw new Error('Missing #app root element');
}

// `?ui=next` opens the workspace shell (driver/pit/tuning/analysis) on the data layer.
// The classic dashboard stays the default until the workspaces cover it.
const ui = new URLSearchParams(window.location.search).get('ui');

mount(ui === 'next' ? NextShell : App, { target });
