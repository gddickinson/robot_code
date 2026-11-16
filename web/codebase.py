"""Scan the repository for the code explorer: which files are shown, their language, size, line count and a summary
taken from the file's own header comment or docstring (never written by hand), plus vendored third-party folders with
the licence each one states.

Pure standard library. The per-project rules (what to skip, which folders are vendored) come from web/project.py.
"""
import ast
import hashlib
import re
from fnmatch import fnmatch
from functools import lru_cache
from pathlib import Path

MAX_BYTES = 200_000  # larger files are listed but not shown

LANGS = [  # (language, extensions or exact file names)
    ("C++", (".ino", ".cpp", ".cc", ".hpp", ".h", ".c", ".pde")),
    ("Python", (".py", ".pyx", ".pxd")),
    ("Shell", (".sh", ".bash", "bashrc_copy", ".bashrc")),
    ("C#", (".cs",)),
    ("XML", (".xml", ".launch", ".xacro", ".urdf", ".sdf", ".world", ".gazebo", ".config", ".dae", ".rviz")),
    ("CMake", ("CMakeLists.txt", ".cmake")),
    ("Markdown", (".md", ".markdown", ".adoc")),
    ("INI", (".ini", ".cfg", ".properties", ".uproject", ".toml")),
    ("ROS msg/srv", (".msg", ".srv", ".action")),
    ("JSON", (".json",)),
    ("YAML", (".yml", ".yaml")),
]
HIGHLIGHT = {"C++": "c", "C#": "c", "Python": "py", "Shell": "sh", "CMake": "sh", "INI": "sh", "XML": "xml"}
BOILERPLATE = ("Fill out your copyright notice",)


def language(path: str) -> str:
    name = path.rsplit("/", 1)[-1]
    for lang, exts in LANGS:
        for e in exts:
            if (e.startswith(".") and name.lower().endswith(e)) or name == e:
                return lang
    return "Text"


def file_id(path: str) -> str:
    """Stable short id for a repo path (used in URLs and static file names)."""
    return hashlib.sha1(path.encode()).hexdigest()[:12]


def read_text(p: Path):
    """File text, or None for binary/undecodable files."""
    raw = p.read_bytes()
    if b"\0" in raw[:8192]:
        return None
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        try:
            text = raw.decode("latin-1")
        except UnicodeDecodeError:
            return None
        return text if sum(c < 32 and c not in (9, 10, 12, 13) for c in raw[:4096]) == 0 else None


def _clean(lines):
    out = [ln.strip() for ln in lines]
    out = [ln for ln in out if not any(b in ln for b in BOILERPLATE)]
    text = " ".join(ln for ln in out if ln).strip()
    return text[:400] + ("..." if len(text) > 400 else "")


def summary(text: str, lang: str) -> str:
    """The file's own header comment / docstring, or '' if it has none."""
    if lang == "Python":
        try:
            doc = ast.get_docstring(ast.parse(text))
        except (SyntaxError, ValueError):
            doc = None
        if doc:
            return _clean(doc.splitlines())
    lines = text.lstrip("﻿").splitlines()
    i = 0
    while i < len(lines) and (not lines[i].strip() or lines[i].startswith("#!")):
        i += 1
    if i >= len(lines):
        return ""
    first = lines[i].strip()
    if lang in ("C++", "C#") or first.startswith(("/*", "//")):
        if first.startswith("/*"):
            block = []
            for ln in lines[i:i + 40]:
                block.append(ln.replace("/*", "").replace("*/", "").strip().lstrip("*").strip())
                if "*/" in ln:
                    break
            return _clean(block)
        block = []
        for ln in lines[i:i + 30]:
            s = ln.strip()
            if not s.startswith("//"):
                break
            block.append(s.lstrip("/").strip())
        return _clean(block)
    if lang in ("Python", "Shell", "CMake", "INI") and first.startswith("#"):
        block = []
        for ln in lines[i:i + 30]:
            s = ln.strip()
            if not s.startswith("#"):
                break
            block.append(s.lstrip("#").strip())
        return _clean(block)
    if lang == "Markdown":
        para = []
        for ln in lines[i:i + 40]:
            s = ln.strip()
            if s.startswith("#") or s.startswith("[!["):
                if para:
                    break
                continue
            if not s and para:
                break
            if s:
                para.append(s)
        return _clean(para)
    if lang == "XML" and "<!--" in first:
        block = []
        for ln in lines[i:i + 20]:
            block.append(ln.replace("<!--", "").replace("-->", ""))
            if "-->" in ln:
                break
        return _clean(block)
    return ""


LICENCE_PATTERNS = [  # most specific first; the earliest match in the text wins (ties go to the earlier pattern)
    ("LGPL-3.0", r"GNU LESSER GENERAL PUBLIC LICENSE\s+Version 3|Lesser General Public\s+License.{0,80}version 3"),
    ("LGPL-2.1", r"GNU LESSER GENERAL PUBLIC LICENSE\s+Version 2\.1|Lesser General Public\s+License.{0,80}version 2\.1"),
    ("LGPL", r"Lesser General Public License|\bLGPL"),
    ("GPL-3.0", r"GNU GENERAL PUBLIC LICENSE\s+Version 3|General Public License.{0,80}version 3|\bGPL\s*v?\.?\s*3|gpl-3\.0"),
    ("GPL-2.0", r"GNU GENERAL PUBLIC LICENSE\s+Version 2|General Public License.{0,80}version 2|\bGPL\s*v?\.?\s*2|gpl-2\.0"),
    ("GPL", r"General Public License|\bGPL\b"),
    ("Apache-2.0", r"Apache License"),
    ("MIT", r"MIT License|Permission is hereby granted, free of charge"),
    ("BSD", r"BSD License|BSD license|Redistribution and use in source and binary forms"),
    ("CC-BY-SA", r"CC-BY-SA|CC BY-SA|Attribution-ShareAlike"),
    ("CC-BY-3.0", r"Creative Commons Attribution 3\.0|CC BY 3\.0"),
    ("Creative Commons", r"Creative Commons"),
    ("Public domain", r"in the public domain"),
]


def detect_licence(text: str):
    best = None
    for i, (name, pat) in enumerate(LICENCE_PATTERNS):
        m = re.search(pat, text, re.I | re.S)
        if m and (best is None or (m.start(), i) < best[0]):
            best = ((m.start(), i), name)
    return best[1] if best else None


def vendored_info(root: Path, rel: str, label: str = "") -> dict:
    """Licence of one vendored folder (or file), from its licence file, metadata, README or source headers."""
    base = root / rel
    files = [base] if base.is_file() else sorted(p for p in base.rglob("*") if p.is_file())
    info = {"path": rel, "name": label or Path(rel).name, "files": len(files),
            "bytes": sum(p.stat().st_size for p in files), "licence": None, "source": None, "author": None, "url": None}
    props = base / "library.properties"
    if props.is_file():
        for ln in (read_text(props) or "").splitlines():
            k, _, v = ln.partition("=")
            if k.strip() in ("author", "url") and v.strip():
                info[k.strip()] = v.strip()

    def rank(p: Path):
        n = p.name.lower()
        if n.startswith(("license", "licence", "copying")):
            return 0
        if n == "library.properties":
            return 1
        if n.startswith("readme"):
            return 2
        if p.suffix.lower() in (".h", ".cpp", ".c", ".py", ".hpp"):
            return 3
        return 4 if p.suffix.lower() in (".ino", ".pde") else 9
    for p in sorted(files, key=lambda p: (rank(p), len(p.parts), p.name)):
        if rank(p) == 9:
            break
        text = read_text(p) if p.stat().st_size < 400_000 else None
        if not text:
            continue
        lic = detect_licence(text if rank(p) < 3 else text[:6000])
        if lic:
            info["licence"], info["source"] = lic, str(p.relative_to(root))
            break
    return info


@lru_cache(maxsize=4)
def scan(root: Path, exclude: tuple, vendored: tuple):
    """Return (shown, skipped): shown = file records with text; skipped = [{path, reason}]."""
    shown, skipped = [], []
    vend_dirs = [v[0] for v in vendored]
    for p in sorted(root.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(root).as_posix()
        if any(part.startswith(".") and part not in (".gitignore",) for part in rel.split("/")[:-1]) \
                or rel.startswith(("site/", "docs/img/")) or "/__pycache__/" in "/" + rel:
            continue
        if any(fnmatch(rel, pat) or fnmatch(p.name, pat) for pat in exclude):
            continue
        if any(rel == v or rel.startswith(v.rstrip("/") + "/") for v in vend_dirs):
            continue
        size = p.stat().st_size
        if size > MAX_BYTES:
            skipped.append({"path": rel, "bytes": size, "reason": "large (over 200 KB) or generated"})
            continue
        text = read_text(p)
        if text is None:
            skipped.append({"path": rel, "bytes": size, "reason": "binary"})
            continue
        lang = language(rel)
        shown.append({"id": file_id(rel), "path": rel, "lang": lang, "hl": HIGHLIGHT.get(lang, ""),
                      "bytes": size, "lines": text.count("\n") + (0 if text.endswith("\n") or not text else 1),
                      "summary": summary(text, lang), "text": text})
    return shown, skipped


def find_line(text: str, needle: str) -> int:
    """1-based line number of the first line containing needle (0 if absent)."""
    for i, ln in enumerate(text.splitlines(), 1):
        if needle in ln:
            return i
    return 0
