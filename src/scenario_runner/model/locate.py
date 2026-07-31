"""Mapping a dotted field path back to a line in the original TOML text.

``tomllib`` discards position information once a document has parsed, so a
validation error found after parsing has no line number of its own. This module
recovers one by scanning the source text for the table header and key that the
field path names. It is a best effort locator: when the exact field cannot be
found (an inline table element, for instance) it walks up the path and reports
the nearest ancestor it can locate, which is still far more useful than no
position at all.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Final

__all__ = ["SourceIndex", "decode_error_line"]

_HEADER: Final[re.Pattern[str]] = re.compile(r"^\s*\[\[?\s*([A-Za-z0-9_.\-]+)\s*\]\]?")
_SEGMENT: Final[re.Pattern[str]] = re.compile(r"([A-Za-z0-9_\-]+)(?:\[(\d+)\])?")
_DECODE_POSITION: Final[re.Pattern[str]] = re.compile(r"at line (\d+)")


@dataclass(frozen=True, slots=True)
class _Segment:
    name: str
    index: int | None


def _parse_field(field: str) -> tuple[_Segment, ...]:
    segments: list[_Segment] = []
    for part in field.split("."):
        match = _SEGMENT.fullmatch(part)
        if match is None:
            return ()
        raw_index = match.group(2)
        segments.append(_Segment(match.group(1), None if raw_index is None else int(raw_index)))
    return tuple(segments)


def decode_error_line(message: str) -> int | None:
    """Extract the line number from a ``tomllib.TOMLDecodeError`` message."""
    match = _DECODE_POSITION.search(message)
    return int(match.group(1)) if match else None


class SourceIndex:
    """Line lookup over the raw text of one scenario document."""

    def __init__(self, text: str) -> None:
        self._lines = text.splitlines()
        self._headers: list[tuple[int, str]] = []
        for number, line in enumerate(self._lines, start=1):
            match = _HEADER.match(line)
            if match is not None:
                self._headers.append((number, match.group(1)))

    def locate(self, field: str) -> int | None:
        """Return a 1-based line number for ``field``, or ``None`` if unlocatable."""
        segments = _parse_field(field)
        while segments:
            line = self._locate_exact(segments)
            if line is not None:
                return line
            segments = segments[:-1]
        return None

    def _locate_exact(self, segments: tuple[_Segment, ...]) -> int | None:
        last = segments[-1]
        if last.index is not None:
            return self._header_line(segments)
        head = segments[:-1]
        span = self._table_span(head)
        if span is None:
            return None
        return self._key_line(last.name, span)

    def _header_line(self, segments: tuple[_Segment, ...]) -> int | None:
        dotted = ".".join(segment.name for segment in segments)
        wanted = segments[-1].index or 0
        seen = 0
        for number, name in self._headers:
            if name != dotted:
                continue
            if seen == wanted:
                return number
            seen += 1
        return None

    def _table_span(self, head: tuple[_Segment, ...]) -> tuple[int, int] | None:
        if not head:
            first_header = self._headers[0][0] if self._headers else len(self._lines) + 1
            return 1, first_header
        start = self._header_line(head)
        if start is None:
            return None
        following = [number for number, _ in self._headers if number > start]
        end = following[0] if following else len(self._lines) + 1
        return start + 1, end

    def _key_line(self, key: str, span: tuple[int, int]) -> int | None:
        pattern = re.compile(rf"^\s*(?:\"{re.escape(key)}\"|{re.escape(key)})\s*=")
        start, end = span
        for number in range(start, min(end, len(self._lines) + 1)):
            if pattern.match(self._lines[number - 1]):
                return number
        return None
