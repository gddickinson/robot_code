// Static-site data layer: answers api(path) calls from the JSON files exported by scripts/build_static.py
// (api/<path>.json). Every page of the showcase is read-only, so nothing needs the Python server.
const files = new Map();
export async function staticApi(path, body) {
  if (body) throw new Error('This action needs the local app (Python server).');
  const clean = path.split('?')[0].replace(/^\/+|\/+$/g, '');
  if (!files.has(clean)) {
    files.set(clean, fetch('api/' + clean + '.json').then(r => { if (!r.ok) throw new Error(`${r.status} ${clean}`); return r.json(); }));
  }
  return files.get(clean);
}
