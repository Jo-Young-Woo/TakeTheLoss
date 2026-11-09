#!/usr/bin/env python3
"""Extract reviewable text and pipe-separated tables from an HTML specification.

Uses only the standard library. No network requests are made.
"""
from __future__ import annotations

import argparse
import hashlib
from html.parser import HTMLParser
from pathlib import Path
import re


class SpecTextParser(HTMLParser):
    OMIT = {"script", "style", "svg", "noscript"}
    BLOCK = {"p", "div", "section", "article", "header", "footer", "nav", "li",
             "h1", "h2", "h3", "h4", "h5", "h6", "pre", "blockquote", "ul", "ol"}

    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.omitted = []
        self.parts = []
        self.row = None
        self.cell = None

    def handle_starttag(self, tag, attrs):
        if self.omitted:
            if tag in self.OMIT:
                self.omitted.append(tag)
            return
        if tag in self.OMIT:
            self.omitted.append(tag)
        elif tag == "tr":
            self.row = []
        elif tag in {"td", "th"}:
            self.cell = []
        elif tag == "br" or tag in self.BLOCK:
            self._append("\n")

    def handle_endtag(self, tag):
        if self.omitted:
            if tag == self.omitted[-1]:
                self.omitted.pop()
            return
        if tag in {"td", "th"} and self.cell is not None:
            value = re.sub(r"\s+", " ", "".join(self.cell)).strip()
            if self.row is not None:
                self.row.append(value)
            self.cell = None
        elif tag == "tr" and self.row is not None:
            self.parts.append("\n| " + " | ".join(self.row) + " |\n")
            self.row = None
        elif tag in self.BLOCK:
            self._append("\n")

    def _append(self, value):
        (self.cell if self.cell is not None else self.parts).append(value)

    def handle_data(self, data):
        if not self.omitted:
            self._append(data)

    def text(self):
        lines = [re.sub(r"[\t \xa0]+", " ", line).strip()
                 for line in "".join(self.parts).splitlines()]
        return "\n".join(line for line in lines if line) + "\n"


def extract_html(source: str) -> str:
    parser = SpecTextParser()
    parser.feed(source)
    parser.close()
    return parser.text()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    raw = args.source.read_bytes()
    text = extract_html(raw.decode("utf-8-sig"))
    header = (f"Source: {args.source.name}\nBytes: {len(raw)}\n"
              f"SHA256: {hashlib.sha256(raw).hexdigest()}\n\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(header + text, encoding="utf-8")
    print(f"Extracted {len(text.splitlines())} content lines to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
