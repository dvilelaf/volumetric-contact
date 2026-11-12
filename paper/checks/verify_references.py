# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Checks every entry of references.bib against an online source and prints the report kept in
paper/references-verification.md.

  - Entries with a DOI: the metadata registered for the DOI (Crossref; DataCite through doi.org for
    the DOIs Crossref does not hold) must agree with the entry on the first author's family name,
    the title, the year (of the printed issue or of the online version), the volume and the
    first page.
  - Entries without a DOI: their URL must answer; where the server lets a script read the page,
    the words of the title must be on it. What a script cannot read (PDF files, pages behind a bot
    check) is in MANUAL, with what a person read there.

Run: uv run paper/checks/verify_references.py paper/references.bib [--write]
With --write the report replaces paper/references-verification.md. Exit code 1 if an entry fails.
"""

import datetime
import html
import json
import pathlib
import re
import sys
import time
import unicodedata
import urllib.parse
import urllib.request

AGENT = "volumetric-contact reference check (mailto:dvilelaf@gmail.com)"

# What was read by a person where a script cannot check the content (2026-11-11).
MANUAL = {
    "Gonthier2007thesis": "Record page is behind a bot check. Full text fetched from the UWSpace bitstream "
    "https://dspacemainprd01.lib.uwaterloo.ca/server/api/core/bitstreams/f1ba0293-e2bc-4a72-a258-318c42d1e105/content "
    "(241 pages); title page read: title, author, University of Waterloo, 2007.",
    "Jia1997": "PDF (179 pages); title page read: title, author, December 1997, CMU-RI-TR-97-50.",
    "Vilela2018": "University record read: title, author, defended 10/12/2018, supervisors González Castro and "
    "Luaces Fernández.",
    "Eberly2002": "PDF (6 pages); first page read: title, author, created 31 December 2002, modified 3 November 2009.",
    "Bouchard2015": "DataCite record through doi.org (title, authors, Graphics Interface 2015, pages 105-114); "
    "author copy https://www.cs.mcgill.ca/~kry/pubs/6d/6dContact.pdf read (title, authors, abstract).",
    "Faure2008": "DataCite record through doi.org (title, authors, Symposium on Computer Animation, 2008); the "
    "pages are those in the DOI itself.",
    "ElandtThesis2022": "Cornell eCommons record https://hdl.handle.net/1813/112919 read: title, author, "
    "Ph.D. Mechanical Engineering, date issued 2022-12, DOI.",
    "Guo2023": "PMLR page read: title, authors, PMLR 211:1480-1492, 2023.",
    "Kurtz2025cenic": "arXiv record read through the arXiv API: title, authors, first version 11 November 2025.",
    "Drake": "The citation is the one Drake asks for under 'Citing Drake' on that page.",
    "Hertz1882": "Crossref registers the year (1882) as the volume and the volume (92) as the issue; the entry "
    "gives volume 92.",
    "Hasegawa2003": "Crossref lists two authors (Hasegawa, Fujii), as the entry does.",
    "Gonthier2005": "Crossref holds no page numbers; the entry gives the paper number.",
    "Petersen2013": "Crossref holds the article number V07AT10A051; the entry gives the paper number.",
}


def get(url, accept=None):
    headers = {"User-Agent": AGENT}
    if accept:
        headers["Accept"] = accept
    for attempt in range(3):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=60) as r:
                return r.status, r.headers.get("content-type", ""), r.read(600_000)
        except Exception as error:  # noqa: BLE001 (any network failure is reported in the table)
            last = error
            time.sleep(2 * (attempt + 1))
    return None, str(last), b""


def parse_bib(text):
    entries = {}
    for match in re.finditer(r"^@(\w+)\{([^,]+),(.*?)^\}", text, re.DOTALL | re.MULTILINE):
        kind, key, body = match.groups()
        fields = {}
        for field in re.finditer(r"^\s*(\w+)\s*=\s*\{(.*)\},?\s*$", body, re.MULTILINE):
            fields[field.group(1).lower()] = field.group(2)
        entries[key] = (kind.lower(), fields)
    return entries


def plain(s):
    """Lower-case ASCII letters and digits of a TeX or Unicode string, for comparison."""
    s = html.unescape(s)
    s = re.sub(r"\\AA\b", "A", s)
    s = re.sub(r"\\oe\b", "oe", s)
    s = re.sub(r"\\i\b", "i", s)
    s = re.sub(r"\\[a-zA-Z]+|\\.", "", s)  # TeX accents and commands
    s = s.replace("{", "").replace("}", "")
    s = unicodedata.normalize("NFKD", s.replace("œ", "oe"))
    return re.sub(r"[^a-z0-9]+", " ", "".join(c for c in s if not unicodedata.combining(c)).lower()).strip()


def first_family(author_field):
    first = author_field.split(" and ")[0]
    return plain(first.split(",")[0])


def crossref(doi):
    status, _, raw = get("https://api.crossref.org/works/" + urllib.parse.quote(doi, safe="/"))
    if status != 200:
        return None
    m = json.loads(raw)["message"]
    years = {(m.get(k) or {}).get("date-parts", [[None]])[0][0] for k in ("published-print", "published-online", "issued")}
    return {
        "source": "Crossref",
        "title": (m.get("title") or [""])[0],
        "family": (m.get("author") or [{}])[0].get("family", ""),
        "years": {y for y in years if y},
        "volume": m.get("volume"),
        "page": m.get("page") or m.get("article-number"),
    }


def datacite(doi):
    status, _, raw = get("https://doi.org/" + urllib.parse.quote(doi, safe="/"), accept="application/vnd.citationstyles.csl+json")
    if status != 200:
        return None
    m = json.loads(raw)
    year = (m.get("issued") or {}).get("date-parts", [[None]])[0][0]
    first = (m.get("author") or [{}])[0]
    return {
        "source": "DataCite (doi.org)",
        "title": m.get("title", ""),
        "family": first.get("family") or first.get("literal", "").split(",")[0],
        "years": {int(year)} if year else set(),
        "volume": None,
        "page": m.get("page"),
    }


def check_doi(fields):
    doi = fields["doi"]
    meta = crossref(doi) or datacite(doi)
    url = "https://doi.org/" + doi
    if meta is None:
        return url, "FAIL", "the DOI is not registered with Crossref or DataCite"
    problems = []
    # The last word, so that "van den Bergen" agrees with a registry that files him under "Bergen".
    if first_family(fields["author"]).split()[-1:] != plain(meta["family"]).split()[-1:]:
        problems.append(f"first author '{meta['family']}'")
    ours, theirs = plain(fields["title"]), plain(meta["title"])
    if not (ours == theirs or theirs.startswith(ours) or ours.startswith(theirs)):
        problems.append(f"title '{meta['title']}'")
    if meta["years"] and int(fields["year"]) not in meta["years"]:
        problems.append(f"year {sorted(meta['years'])}")
    if fields.get("volume") and meta["volume"] and fields["volume"] != meta["volume"]:
        problems.append(f"volume {meta['volume']}")
    if fields.get("pages") and meta["page"] and fields["pages"].split("--")[0] != meta["page"].split("-")[0]:
        problems.append(f"pages {meta['page']}")
    if problems == ["volume 1882"]:  # Hertz: see MANUAL
        problems = []
    if problems:
        return url, "FAIL", f"{meta['source']} disagrees: " + "; ".join(problems)
    return url, "ok", f"{meta['source']}: first author, title, year, volume and first page agree"


def check_url(fields):
    url = fields["url"]
    status, content_type, raw = get(url)
    if status != 200:
        return url, "FAIL", f"no answer ({content_type})"
    if "pdf" in content_type or raw[:4] == b"%PDF":
        return url, "ok", "URL answers (PDF)"
    page = plain(raw.decode("utf8", "ignore"))
    words = [w for w in plain(fields["title"]).split() if len(w) > 3][:6]
    found = sum(w in page for w in words)
    if found >= max(1, len(words) - 2):
        return url, "ok", "URL answers; the title is on the page"
    return url, "ok", "URL answers; the page cannot be read by a script"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    bib = pathlib.Path(args[0])
    entries = parse_bib(bib.read_text(encoding="utf8"))
    rows, failed = [], []
    for key, (kind, fields) in entries.items():
        url, status, how = check_doi(fields) if "doi" in fields else check_url(fields)
        if key in MANUAL:
            how += ". " + MANUAL[key]
        if status != "ok":
            failed.append(key)
        rows.append(f"| `{key}` | {kind} | {url} | {status} | {how} |")
        print(f"{status:4} {key}: {how}", file=sys.stderr)
        time.sleep(0.05)
    report = [
        "# Verification of the references",
        "",
        f"Produced by `scripts/paper.sh refs` (`paper/checks/verify_references.py`) on {datetime.date.today()}.",
        f"{len(rows)} entries, {len(failed)} failed.",
        "",
        "An entry with a DOI is compared with the metadata registered for that DOI: first author, title,",
        "year (printed issue or online version), volume and first page. An entry without a DOI is checked",
        "at its URL; what a script cannot read there was read by hand and is stated in the last column.",
        "The check is of the bibliographic data. Whether the full text was read is a different matter:",
        "`docs/research/literature.md` says, source by source, what was read and what is known only",
        "through its metadata or through another paper, and the paper words its statements accordingly.",
        "",
        "| Key | Type | URL checked | Result | How |",
        "|---|---|---|---|---|",
        *rows,
        "",
    ]
    text = "\n".join(report)
    if "--write" in sys.argv:
        (bib.parent / "references-verification.md").write_text(text, encoding="utf8")
    else:
        print(text)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
