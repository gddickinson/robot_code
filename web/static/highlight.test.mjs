// Node test for the code-explorer highlighter: lossless tokenisation and the expected classes on small snippets.
// Run: node web/static/highlight.test.mjs
import assert from 'node:assert/strict';
import { readFileSync, readdirSync, statSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { tokenize, lines } from './highlight.js';

const join_ = t => t.map(x => x[1]).join('');
const cls = (t, piece) => (t.find(x => x[1] === piece) || [])[0];

let t = tokenize('// hi\nint x = 0x1F; /* a\nb */ char c = \'$\';\n#include <Wire.h>\n', 'c');
assert.equal(join_(t), '// hi\nint x = 0x1F; /* a\nb */ char c = \'$\';\n#include <Wire.h>\n');
assert.equal(cls(t, '// hi'), 'com'); assert.equal(cls(t, 'int'), 'kw'); assert.equal(cls(t, '0x1F'), 'num');
assert.equal(cls(t, '/* a\nb */'), 'com'); assert.equal(cls(t, "'$'"), 'str'); assert.equal(cls(t, '#include <Wire.h>'), 'pre');

t = tokenize('def f(x):\n    """doc"""\n    return x  # c\n', 'py');
assert.equal(cls(t, 'def'), 'kw'); assert.equal(cls(t, '"""doc"""'), 'str'); assert.equal(cls(t, '# c'), 'com');

t = tokenize('#!/bin/bash\nvar=$(ip addr)\necho "IP: $var" # x\nexport A=http://$var:11311\n', 'sh');
assert.equal(cls(t, '#!/bin/bash'), 'com'); assert.equal(cls(t, '"IP: $var"'), 'str'); assert.equal(cls(t, '$var'), 'var');
assert.equal(cls(t, 'export'), 'kw');

t = tokenize('<?xml version="1.0"?>\n<launch><!-- c --><node name="a"/></launch>', 'xml');
assert.equal(cls(t, '<!-- c -->'), 'com'); assert.equal(cls(t, '<node'), 'tag'); assert.equal(cls(t, 'name'), 'attr');

assert.deepEqual(lines('a\n<b>\n', 'c'), ['a', '&lt;b&gt;']);
assert.equal(lines('/* x\ny */', 'c').length, 2);

// lossless on every source file in the repo that the explorer would highlight
const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const HL = { '.ino': 'c', '.cpp': 'c', '.h': 'c', '.c': 'c', '.cs': 'c', '.py': 'py', '.pyx': 'py', '.sh': 'sh', '.launch': 'xml', '.xml': 'xml', '.xacro': 'xml' };
let n = 0;
function walk(dir) {
  for (const name of readdirSync(dir)) {
    if (['.git', 'node_modules', 'site', 'libraries'].includes(name)) continue;
    const p = join(dir, name), st = statSync(p);
    if (st.isDirectory()) { walk(p); continue; }
    const ext = name.slice(name.lastIndexOf('.'));
    if (!HL[ext] || st.size > 200000) continue;
    const text = readFileSync(p, 'latin1');
    assert.equal(join_(tokenize(text, HL[ext])), text, p);
    n++;
  }
}
walk(root);
console.log(`highlight ok (${n} repo files round-trip)`);
