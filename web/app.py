"""FastAPI web app for the project showcase: serves the front end in static/ and the JSON API under /api.

Run with `python run_web.py` (local only). `scripts/build_static.py` exports every path in STATIC_ENDPOINTS to JSON
for the GitHub Pages site, so the static site shows exactly what the server does. Project-specific content (title,
what to skip, vendored folders, "How it works" sections, demo endpoints) lives in web/project.py.
"""
from collections import Counter
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from . import codebase, project

STATIC = Path(__file__).resolve().parent / "static"
ROOT = Path(__file__).resolve().parent.parent

app = FastAPI(title=project.TITLE)
app.mount("/static", StaticFiles(directory=STATIC), name="static")


def files():
    return codebase.scan(ROOT, tuple(project.EXCLUDE), tuple(project.VENDORED))


def by_path():
    return {f["path"]: f for f in files()[0]}


@app.get("/", include_in_schema=False)
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/info")
def info():
    """Title, description and real counts for the home page."""
    shown, skipped = files()
    own = [f for f in shown if f["lang"] not in project.NOT_CODE]
    langs = Counter()
    for f in own:
        langs[f["lang"]] += f["lines"]
    return {"title": project.TITLE, "repo": project.REPO, "description": project.DESCRIPTION,
            "kicker": project.KICKER, "files": len(shown), "code_files": len(own),
            "lines": sum(langs.values()), "langs": langs.most_common(), "skipped": len(skipped),
            "vendored": len(project.VENDORED), "tiles": project.tiles(shown), "demo": project.DEMO}


@app.get("/api/tree")
def tree():
    """Every shown file (without its text), files left out and why, and the vendored folders."""
    shown, skipped = files()
    return {"files": [{k: v for k, v in f.items() if k != "text"} for f in shown], "skipped": skipped,
            "vendored": [{"path": p, "name": n} for p, n in project.VENDORED]}


@app.get("/api/file/{fid}")
def file(fid: str):
    """One file's text and metadata."""
    for f in files()[0]:
        if f["id"] == fid:
            return f
    raise HTTPException(404, "no such file")


@app.get("/api/corpus")
def corpus():
    """All shown text, for searching inside files in the browser."""
    return [{"id": f["id"], "path": f["path"], "text": f["text"]} for f in files()[0]]


@app.get("/api/vendored")
def vendored():
    """Third-party folders kept in the repo, with the licence each one states (or None if none is found)."""
    return [codebase.vendored_info(ROOT, p, n) for p, n in project.VENDORED]


def resolve(cite):
    """A citation {file, find, note} -> with the line number where `find` occurs, the line text and the file id."""
    f = by_path().get(cite["file"])
    if f is None:
        raise KeyError(f"cited file not shown: {cite['file']}")
    line = codebase.find_line(f["text"], cite["find"])
    if not line:
        raise KeyError(f"{cite['find']!r} not found in {cite['file']}")
    return {**cite, "id": f["id"], "line": line, "code": f["text"].splitlines()[line - 1].strip()}


@app.get("/api/how")
def how():
    """The 'How it works' sections, each claim cited to a file and line resolved from the code itself."""
    return [{**s, "cites": [resolve(c) for c in s.get("cites", [])]} for s in project.HOW]


@app.get("/api/about")
def about():
    return project.ABOUT


project.register(app)

# GET paths (without the /api prefix) written to the static site as api/<path>.json.
STATIC_ENDPOINTS = ["/info", "/tree", "/corpus", "/vendored", "/how", "/about"] \
    + [f"/file/{f['id']}" for f in files()[0]] + list(project.EXTRA_ENDPOINTS)
