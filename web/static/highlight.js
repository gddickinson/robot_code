// Small dependency-free syntax highlighter for the code explorer. tokenize(text, lang) splits the text into
// [cls, piece] tokens whose pieces join back to the exact input (lossless; tested in highlight.test.mjs);
// lines(text, lang) returns one HTML string per source line. Languages: 'c' (C, C++, Arduino, C#), 'py', 'sh', 'xml';
// anything else is shown plain.

const C_KW = new Set(('auto bool boolean break byte case catch char class const constexpr continue default delete do double else enum ' +
  'explicit extern false float for friend goto if inline int long namespace new nullptr operator private protected public ' +
  'register return short signed sizeof static static_cast struct switch template this throw true try typedef typename ' +
  'union unsigned using virtual void volatile while word uint8_t uint16_t uint32_t int8_t int16_t int32_t int64_t uint64_t ' +
  'size_t String override const_cast dynamic_cast reinterpret_cast HIGH LOW INPUT OUTPUT INPUT_PULLUP string var foreach in').split(' '));
const PY_KW = new Set(('and as assert async await break class continue def del elif else except False finally for from global if ' +
  'import in is lambda None nonlocal not or pass print raise return self True try while with yield cdef cimport cpdef').split(' '));
const SH_KW = new Set(('if then else elif fi for while until do done case esac in function return export local read echo source ' +
  'sudo cd alias shopt set unset exit').split(' '));

const RULES = {
  c: [
    ['com', /\/\/[^\n]*|\/\*[\s\S]*?(?:\*\/|$)/y],
    ['str', /"(?:\\.|[^"\\\n])*"?|'(?:\\.|[^'\\\n])*'?/y],
    ['pre', /#[ \t]*[a-z]+\b[^\n]*/y],
    ['num', /\b(?:0x[0-9a-fA-F]+|0b[01]+|\d+\.?\d*(?:[eE][-+]?\d+)?[fFuUlL]*)\b/y],
    ['id', /[A-Za-z_]\w*/y],
  ],
  py: [
    ['com', /#[^\n]*/y],
    ['str', /[rbuRBUfF]{0,2}(?:"""[\s\S]*?(?:"""|$)|'''[\s\S]*?(?:'''|$)|"(?:\\.|[^"\\\n])*"?|'(?:\\.|[^'\\\n])*'?)/y],
    ['num', /\b(?:0x[0-9a-fA-F]+|\d+\.?\d*(?:[eE][-+]?\d+)?)\b/y],
    ['dec', /@[\w.]+/y],
    ['id', /[A-Za-z_]\w*/y],
  ],
  sh: [
    ['com', /(?:^|(?<=[\s;]))#[^\n]*/y],
    ['str', /"(?:\\.|[^"\\])*"?|'[^']*'?/y],
    ['var', /\$(?:\{[^}\n]*\}?|\w+|[@#?$!*])/y],
    ['num', /\b\d+\b/y],
    ['id', /[A-Za-z_][\w-]*/y],
  ],
  xml: [
    ['com', /<!--[\s\S]*?(?:-->|$)/y],
    ['pre', /<\?[\s\S]*?(?:\?>|$)/y],
    ['tag', /<\/?[\w:.-]+|\/?>/y],
    ['attr', /[\w:.-]+(?==)/y],
    ['str', /"[^"]*"?|'[^']*'?/y],
  ],
};
const KW = { c: C_KW, py: PY_KW, sh: SH_KW };

export function tokenize(text, lang) {
  const rules = RULES[lang];
  if (!rules) return [['', text]];
  const out = [];
  let i = 0, plain = '';
  const flush = () => { if (plain) { out.push(['', plain]); plain = ''; } };
  while (i < text.length) {
    let hit = null;
    for (const [cls, re] of rules) {
      re.lastIndex = i;
      const m = re.exec(text);
      if (m && m[0].length) { hit = [cls, m[0]]; break; }
    }
    if (!hit) { plain += text[i++]; continue; }
    let [cls, piece] = hit;
    if (cls === 'id') cls = KW[lang] && KW[lang].has(piece) ? 'kw' : '';
    if (cls === 'pre' && lang === 'c' && !/^#/.test(piece)) cls = '';
    if (cls) { flush(); out.push([cls, piece]); } else plain += piece;
    i += piece.length;
  }
  flush();
  return out;
}

const escHtml = s => s.replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

// One HTML string per line (spans are closed at line ends so each line can sit in its own row).
export function lines(text, lang) {
  const rows = [''];
  for (const [cls, piece] of tokenize(text, lang)) {
    piece.split('\n').forEach((part, k) => {
      if (k) rows.push('');
      if (part) rows[rows.length - 1] += cls ? `<span class="h-${cls}">${escHtml(part)}</span>` : escHtml(part);
    });
  }
  if (rows.length > 1 && rows[rows.length - 1] === '' && text.endsWith('\n')) rows.pop();
  return rows;
}
