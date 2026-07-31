"""Located errors raised while reading a scenario document."""

from __future__ import annotations

__all__ = ["ScenarioError"]


class ScenarioError(ValueError):
    """A scenario document is malformed, with the offending field located.

    The string form is ``source:line: field: message`` so an editor or a log
    reader can jump straight to the problem. ``line`` is omitted when the
    offending field could not be located in the source text, and ``field`` is
    omitted for document-level problems such as a syntax error.
    """

    def __init__(
        self,
        message: str,
        *,
        source: str = "<scenario>",
        field: str = "",
        line: int | None = None,
    ) -> None:
        self.reason = message
        self.source = source
        self.field = field
        self.line = line
        super().__init__(self._render())

    def _render(self) -> str:
        where = self.source if self.line is None else f"{self.source}:{self.line}"
        if self.field:
            return f"{where}: {self.field}: {self.reason}"
        return f"{where}: {self.reason}"
