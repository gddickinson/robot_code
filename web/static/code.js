// Code explorer: folder tree with a filename filter, text search across every shown file, and a source viewer with
// line numbers and highlighting. Routes: #/code, #/code/<file id>?line=N, #/code?q=text (search inside files).
import { route, api, app, esc, list } from './core.js';
import { lines } from './highlight.js';

const fmtKB = b => b < 1024 ? `${b} B` : `${(b / 1024).toFixed(1)} KB`;

function treeHtml(files, current) {
  const groups = new Map();
  for (const f of files) {
    const cut = f.path.lastIndexOf('/');
    const dir = cut < 0 ? '' : f.path.slice(0, cut);
    if (!groups.has(dir)) groups.set(dir, []);
    groups.get(dir).push(f);
  }
  const link = f => `<a class="tf${f.id === current ? ' on' : ''}" href="#/code/${f.id}" title="${esc(f.path)}">${esc(f.path.split('/').pop())}<i>${f.lines}</i></a>`;
  return list([...groups.keys()].sort(), dir => dir === ''
    ? list(groups.get(dir), link)
    : `<details${groups.get(dir).some(f => f.id === current) || groups.size < 9 ? ' open' : ''}><summary>${esc(dir)}/ <i>${groups.get(dir).length}</i></summary>${list(groups.get(dir), link)}</details>`);
}

function searchText(corpus, q, max = 200) {
  const hits = [], needle = q.toLowerCase();
  for (const f of corpus) {
    const rows = f.text.split('\n');
    for (let i = 0; i < rows.length && hits.length < max; i++) {
      if (rows[i].toLowerCase().includes(needle)) hits.push({ id: f.id, path: f.path, line: i + 1, code: rows[i].trim() });
    }
  }
  return hits;
}

function viewer(f, line) {
  const rows = lines(f.text, f.hl);
  return `<div class="fhead"><h2>${esc(f.path)}</h2>
    <p class="small mute">${esc(f.lang)} · ${f.lines} lines · ${fmtKB(f.bytes)}</p>
    ${f.summary ? `<p class="fsum"><span class="small mute">From the file's header:</span> ${esc(f.summary)}</p>` : ''}</div>
    <div class="src" role="region" aria-label="Source of ${esc(f.path)}" tabindex="0"><table><tbody>${rows.map((r, i) =>
      `<tr id="L${i + 1}"${i + 1 === line ? ' class="hit"' : ''}><td class="ln">${i + 1}</td><td class="lc">${r || ' '}</td></tr>`).join('')}</tbody></table></div>`;
}

route('code', async (args, query) => {
  const t = await api('/tree');
  const fid = args[0] || null, line = +query.line || 0;
  app().innerHTML = `<p class="kicker">Code explorer</p><h1>Source files</h1>
  <p class="lead">${t.files.length} files shown${t.skipped.length ? `, ${t.skipped.length} left out (binary, generated or over 200 KB)` : ''}${t.vendored.length ? `; ${t.vendored.length} vendored third-party folders are listed on the <a href="#/about">About</a> page instead` : ''}.</p>
  <div class="explorer">
    <aside class="panel tree">
      <label for="cf">Filter by file name</label><input id="cf" type="search" placeholder="e.g. motor">
      <label for="cq">Search inside files</label><input id="cq" type="search" value="${esc(query.q || '')}" placeholder="e.g. Serial.read">
      <div id="ctree" role="navigation" aria-label="Files"></div>
    </aside>
    <section id="cview" class="cview"></section>
  </div>`;
  const tree = document.getElementById('ctree'), view = document.getElementById('cview');
  const cf = document.getElementById('cf'), cq = document.getElementById('cq');
  const drawTree = () => {
    const q = cf.value.trim().toLowerCase();
    const shown = q ? t.files.filter(f => f.path.toLowerCase().includes(q)) : t.files;
    tree.innerHTML = shown.length ? treeHtml(shown, fid) : '<p class="small mute">No file names match.</p>';
  };
  cf.addEventListener('input', drawTree);
  drawTree();

  const overview = () => {
    view.innerHTML = `<h2>Files</h2><div class="scroll"><table class="flist"><thead><tr><th>File</th><th>Lines</th><th>Summary (from the file)</th></tr></thead><tbody>
      ${list(t.files, f => `<tr><td><a href="#/code/${f.id}">${esc(f.path)}</a><br><span class="tag">${esc(f.lang)}</span></td><td>${f.lines}</td><td class="small">${esc(f.summary || '')}</td></tr>`)}
      </tbody></table></div>
      ${t.skipped.length ? `<h3>Left out</h3><ul class="small">${list(t.skipped, s => `<li>${esc(s.path)} <span class="mute">(${esc(s.reason)}, ${fmtKB(s.bytes)})</span></li>`)}</ul>` : ''}`;
  };
  let seq = 0;
  const runSearch = async () => {
    const q = cq.value.trim(), n = ++seq;
    history.replaceState(null, '', '#/code' + (fid ? '/' + fid : '') + (q ? '?q=' + encodeURIComponent(q) : ''));
    if (!q) { return fid ? openFile() : overview(); }
    if (q.length < 2) return;
    const hits = searchText(await api('/corpus'), q);
    if (n !== seq) return;
    view.innerHTML = `<h2>${hits.length >= 200 ? '200+' : hits.length} matches for "${esc(q)}"</h2>
      <div class="hits">${list(hits, h => `<a href="#/code/${h.id}?line=${h.line}"><b>${esc(h.path)}:${h.line}</b><code>${esc(h.code.slice(0, 160))}</code></a>`)}</div>`;
  };
  const openFile = async () => {
    const f = await api('/file/' + fid);
    view.innerHTML = viewer(f, line);
    if (line) document.getElementById('L' + line)?.scrollIntoView({ block: 'center' });
  };
  cq.addEventListener('input', runSearch);
  if (query.q) await runSearch();
  else if (fid) await openFile();
  else overview();
});
