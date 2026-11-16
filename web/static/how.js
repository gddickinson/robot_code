// How it works: sections written from the code (web/project.py HOW), each with its citations resolved by the server
// to the exact file and line, linking into the code explorer.
import { route, api, app, esc, list } from './core.js';

route('how', async () => {
  const secs = await api('/how');
  app().innerHTML = `<p class="kicker">How it works</p><h1>The mechanism, from the code</h1>
  <p class="lead">Every statement below is cited to the file and line it comes from; click a citation to open the source there.</p>
  <nav class="toc-inline">${list(secs, s => `<a href="#/how?keep=1" data-sec="${esc(s.id)}">${esc(s.title)}</a>`)}</nav>
  ${list(secs, s => `<section class="how panel" id="sec-${esc(s.id)}">
    <h2>${esc(s.title)}</h2>
    <div class="prose">${s.body}</div>
    ${s.svg ? `<figure class="diagram">${s.svg}${s.caption ? `<figcaption class="small mute">${esc(s.caption)}</figcaption>` : ''}</figure>` : ''}
    ${s.cites.length ? `<h3>Sources</h3><ul class="cites">${list(s.cites, c => `<li><a href="#/code/${c.id}?line=${c.line}">${esc(c.file)}:${c.line}</a>
      ${c.note ? `<span class="small">${esc(c.note)}</span>` : ''}<code>${esc(c.code.slice(0, 140))}</code></li>`)}</ul>` : ''}
  </section>`)}`;
  app().querySelectorAll('[data-sec]').forEach(a => a.addEventListener('click', e => {
    e.preventDefault();
    document.getElementById('sec-' + a.dataset.sec)?.scrollIntoView({ behavior: 'smooth' });
  }));
});
