// Home (hero, real counts from the repo scan, entry cards) and About (from web/project.py ABOUT + vendored licences).
import { route, api, app, esc, list } from './core.js';

const fmt = n => Number(n).toLocaleString('en-GB');

route('home', async () => {
  const s = await api('/info');
  const doors = [
    ['#/code', 'Code explorer', `Browse and search all ${fmt(s.files)} files: syntax-highlighted source with line numbers and each file's own header comment.`],
    ['#/how', 'How it works', 'The mechanism explained from the code, with every claim linked to the file and line it comes from.'],
    s.demo ? ['#/demo', s.demo.title, s.demo.blurb] : null,
    ['#/about', 'About', 'What you need to run the real thing, credits, licences and third-party code kept in the repo.'],
  ].filter(Boolean);
  app().innerHTML = `
  <section class="hero">
    <p class="kicker">${esc(s.kicker)}</p>
    <h1 class="glow">${esc(s.title)}</h1>
    <p class="lead">${esc(s.description)}</p>
    <p><a class="btn" href="#/code">Explore the code</a><a class="btn alt" href="#/how">How it works</a>${s.demo ? '<a class="btn alt" href="#/demo">Try the demo</a>' : ''}</p>
  </section>
  <div class="tiles">
    ${list(s.tiles, ([n, label]) => `<div class="tile"><b>${fmt(n)}</b><span>${esc(label)}</span></div>`)}
    <div class="tile"><b>${fmt(s.lines)}</b><span>lines of own code</span></div>
    ${list(s.langs.slice(0, 3), ([lang, n]) => `<div class="tile"><b>${fmt(n)}</b><span>lines of ${esc(lang)}</span></div>`)}
    ${s.vendored ? `<a class="tile" href="#/about"><b>${fmt(s.vendored)}</b><span>vendored third-party folders</span></a>` : ''}
  </div>
  <div class="grid wide">${list(doors, ([href, t, d]) => `<a class="card door" href="${href}"><b>${esc(t)}</b><span>${esc(d)}</span></a>`)}</div>`;
});

route('about', async () => {
  const [a, v, s] = await Promise.all([api('/about'), api('/vendored'), api('/info')]);
  const sec = (title, items) => items && items.length ? `<h2>${esc(title)}</h2><ul>${list(items, i => `<li>${i}</li>`)}</ul>` : '';
  app().innerHTML = `<p class="kicker">About</p><h1>${esc(s.title)}</h1>
  <div class="prose">${a.what}</div>
  ${sec('To run the real thing', a.needs)}
  ${sec('Running it', a.run)}
  <h2>This web site</h2>
  <p>Run locally with <code>pip install -r requirements-web.txt && python run_web.py</code>
  (<a href="http://127.0.0.1:${a.port}/">http://127.0.0.1:${a.port}</a>). <code>python scripts/build_static.py</code> builds the
  static GitHub Pages copy, which shows the same pages from exported JSON. Everything shown is read from the repository
  itself; nothing is invented.</p>
  ${sec('Credits', a.credits)}
  <h2>Licence</h2><p>${a.licence}</p>
  ${v.length ? `<h2>Third-party code kept in the repo</h2>
  <p class="small mute">These folders are other people's work, kept so the code builds; they are not shown in the code explorer
  and are not counted as this project's code. The licence column is what the folder itself states
  (file named in the last column); "not stated" means no licence text was found in it.</p>
  <div class="scroll"><table><thead><tr><th>Folder</th><th>Files</th><th>Licence</th><th>Author / URL</th><th>Stated in</th></tr></thead><tbody>
  ${list(v, x => `<tr><td><b>${esc(x.name)}</b><br><span class="small mute">${esc(x.path)}</span></td><td>${x.files}</td>
    <td>${x.licence ? `<span class="tag a">${esc(x.licence)}</span>` : '<span class="mute">not stated</span>'}</td>
    <td class="small">${esc(x.author || '')}${x.url ? `<br><span class="mute">${esc(x.url)}</span>` : ''}</td>
    <td class="small mute">${esc(x.source || '')}</td></tr>`)}
  </tbody></table></div>` : ''}`;
});
