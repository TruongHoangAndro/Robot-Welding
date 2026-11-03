"""Strict reader / writer for the small YAML subset of the cell role map and ``frames.yaml``.

The conda env ``awm_occ`` has no PyYAML and nothing is installed from this package's tasks (AGENT_TEAM.md §2.6).
Supported: comments, block mappings, block sequences (also of mappings: ``- key: value``), flow sequences of
scalars (``[1, 2]``), single / double quoted and plain scalars (null, true / false, int, float, string).
Anything else (anchors, tags, multi-line or block scalars, flow mappings, nested flow sequences) is an error, never
a silent misread. Replace by PyYAML once the env provides it (request in the A3 report).
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, List, Tuple, Union

_INT = re.compile(r'^[-+]?\d+$')
_FLOAT = re.compile(r'^[-+]?(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?$')
_UNSUPPORTED = ('&', '*', '!', '|', '>', '{', '%', '@', '`')


class YamlSubsetError(ValueError):
    """The text uses YAML outside the supported subset, or is malformed."""


_TOKEN_START = (':', '-', '[', ',', '?')


def _closing_quote(text: str, i: int, lineno: int) -> int:
    """Index of the quote that closes the quoted token opening at ``text[i]``."""
    quote, j = text[i], i + 1
    while j < len(text):
        ch = text[j]
        if quote == "'" and ch == "'":
            if j + 1 < len(text) and text[j + 1] == "'":
                j += 2
                continue
            return j
        if quote == '"' and ch == '\\':
            j += 2
            continue
        if quote == '"' and ch == '"':
            return j
        j += 1
    raise YamlSubsetError(f'line {lineno}: unterminated quote: {text!r}')


def _opens_quote(text: str, i: int) -> bool:
    """A quote opens a quoted token only at the start of a token (line start, after ': ', '- ', '[', ',')."""
    if text[i] not in ('"', "'"):
        return False
    prev = text[:i].rstrip()
    return prev == '' or prev[-1] in _TOKEN_START


def _quote_spans(text: str, lineno: int) -> List[Tuple[int, int]]:
    spans, i = [], 0
    while i < len(text):
        if _opens_quote(text, i):
            j = _closing_quote(text, i, lineno)
            spans.append((i, j))
            i = j + 1
        else:
            i += 1
    return spans


def _outside(spans: List[Tuple[int, int]], i: int) -> bool:
    return not any(a <= i <= b for a, b in spans)


def _strip_comment(line: str, lineno: int) -> str:
    i = 0
    while i < len(line):
        if _opens_quote(line, i):
            i = _closing_quote(line, i, lineno) + 1
            continue
        if line[i] == '#' and (i == 0 or line[i - 1] in ' \t'):
            return line[:i].rstrip()
        i += 1
    return line.rstrip()


def _split_outside_quotes(text: str, sep: str, lineno: int) -> List[str]:
    spans = _quote_spans(text, lineno)
    parts, start = [], 0
    for i, ch in enumerate(text):
        if ch == sep and _outside(spans, i):
            parts.append(text[start:i])
            start = i + 1
    parts.append(text[start:])
    return parts


def _key_value(text: str, lineno: int) -> Tuple[str, str]:
    spans = _quote_spans(text, lineno)
    for i, ch in enumerate(text):
        if ch == ':' and _outside(spans, i) and (i + 1 == len(text) or text[i + 1] == ' '):
            key = _scalar(text[:i].strip(), lineno)
            if not isinstance(key, str):
                key = str(key)
            return key, text[i + 1:].strip()
    raise YamlSubsetError(f'line {lineno}: expected "key: value": {text!r}')


def _scalar(text: str, lineno: int) -> Any:
    if text == '':
        return None
    if text[0] in ('"', "'"):
        if _quote_spans(text, lineno) != [(0, len(text) - 1)]:
            raise YamlSubsetError(f'line {lineno}: text after a closing quote: {text!r}')
        if text[0] == "'":
            return text[1:-1].replace("''", "'")
        try:
            return json.loads(text)
        except ValueError as exc:
            raise YamlSubsetError(f'line {lineno}: bad double-quoted string {text!r}') from exc
    if text[0] == '[':
        if text[-1] != ']':
            raise YamlSubsetError(f'line {lineno}: unterminated flow sequence: {text!r}')
        inner = text[1:-1].strip()
        if not inner:
            return []
        items = [p.strip() for p in _split_outside_quotes(inner, ',', lineno)]
        if any(item == '' for item in items):
            raise YamlSubsetError(f'line {lineno}: empty item in flow sequence: {text!r}')
        if any(item[0] == '[' for item in items):
            raise YamlSubsetError(f'line {lineno}: nested flow sequences are not supported')
        return [_scalar(item, lineno) for item in items]
    if text[0] in _UNSUPPORTED or text[0] in ',]}?' or text == '-' or text.startswith('- '):
        raise YamlSubsetError(f'line {lineno}: unsupported YAML syntax: {text!r}')
    if any(ch in text for ch in '[]{}'):
        raise YamlSubsetError(f'line {lineno}: bracket in a plain scalar (stray or nested flow syntax): {text!r}')
    if text in ('null', '~', 'Null', 'NULL'):
        return None
    if text in ('true', 'True', 'TRUE'):
        return True
    if text in ('false', 'False', 'FALSE'):
        return False
    if _INT.match(text):
        return int(text)
    if _FLOAT.match(text):
        return float(text)
    if ': ' in text or text.endswith(':'):
        raise YamlSubsetError(f'line {lineno}: unexpected mapping in a scalar: {text!r}')
    return text


def loads(text: str) -> Any:
    """Parse ``text`` (the supported subset) into dicts, lists and scalars."""
    lines: List[Tuple[int, str, int]] = []
    seen_marker = False
    for n, raw in enumerate(text.splitlines(), start=1):
        if '\t' in raw[:len(raw) - len(raw.lstrip())]:
            raise YamlSubsetError(f'line {n}: tab indentation')
        body = _strip_comment(raw, n)
        if body.strip() == '':
            continue
        if body.strip() == '---':
            if lines or seen_marker:
                raise YamlSubsetError(f'line {n}: only one document is supported')
            seen_marker = True
            continue
        if body.strip()[0] == '?':
            raise YamlSubsetError(f'line {n}: complex keys ("?") are not supported')
        lines.append((len(body) - len(body.lstrip(' ')), body.strip(), n))
    if not lines:
        return None
    pos = [0]

    def block(indent: int) -> Any:
        ind, content, n = lines[pos[0]]
        if ind != indent:
            raise YamlSubsetError(f'line {n}: bad indentation')
        if content == '-' or content.startswith('- '):
            return sequence(indent)
        if ':' not in content:
            pos[0] += 1
            if pos[0] < len(lines) and lines[pos[0]][0] >= indent:
                raise YamlSubsetError(f'line {n}: a scalar cannot be followed by an indented block')
            return _scalar(content, n)
        return mapping(indent)

    def child(parent_indent: int, n: int, allow_same_indent_seq: bool) -> Any:
        if pos[0] >= len(lines):
            return None
        ind, content, _ = lines[pos[0]]
        if ind > parent_indent:
            return block(ind)
        if allow_same_indent_seq and ind == parent_indent and (content == '-' or content.startswith('- ')):
            return sequence(ind)
        return None

    def mapping(indent: int) -> dict:
        out: dict = {}
        while pos[0] < len(lines):
            ind, content, n = lines[pos[0]]
            if ind < indent:
                break
            if ind > indent:
                raise YamlSubsetError(f'line {n}: bad indentation')
            if content == '-' or content.startswith('- '):
                break
            key, value = _key_value(content, n)
            if key in out:
                raise YamlSubsetError(f'line {n}: duplicate key {key!r}')
            pos[0] += 1
            out[key] = _scalar(value, n) if value else child(indent, n, True)
        return out

    def sequence(indent: int) -> list:
        out: list = []
        while pos[0] < len(lines):
            ind, content, n = lines[pos[0]]
            if ind != indent or not (content == '-' or content.startswith('- ')):
                if ind > indent:
                    raise YamlSubsetError(f'line {n}: bad indentation')
                break
            item = content[1:].strip()
            if not item:
                pos[0] += 1
                out.append(child(indent, n, False))
            elif (item[0] not in '\'"[') and re.search(r':( |$)', item):
                # "- key: value" starts a mapping whose keys sit at indent + 2
                lines[pos[0]] = (indent + 2, item, n)
                out.append(mapping(indent + 2))
            else:
                pos[0] += 1
                out.append(_scalar(item, n))
        return out

    result = block(lines[0][0])
    if pos[0] != len(lines):
        raise YamlSubsetError(f'line {lines[pos[0]][2]}: unexpected content')
    return result


def load(path: Union[str, Path]) -> Any:
    return loads(Path(path).read_text())


def _dump_scalar(value: Any) -> str:
    if value is None:
        return 'null'
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return repr(float(value)) if isinstance(value, float) else str(value)
    return json.dumps(str(value), ensure_ascii=False)


def dumps(data: Any, indent: int = 0) -> str:
    """Block-style YAML of dicts / lists / scalars; lists of scalars in flow style."""
    pad = ' ' * indent
    out: List[str] = []
    if isinstance(data, dict):
        for key, value in data.items():
            k = _dump_scalar(str(key)) if not re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', str(key)) else str(key)
            if isinstance(value, dict) and value:
                out.append(f'{pad}{k}:')
                out.append(dumps(value, indent + 2))
            elif isinstance(value, list) and value and any(isinstance(v, (dict, list)) for v in value):
                out.append(f'{pad}{k}:')
                out.append(dumps(value, indent + 2))
            elif isinstance(value, list):
                out.append(f'{pad}{k}: [' + ', '.join(_dump_scalar(v) for v in value) + ']')
            elif isinstance(value, dict):
                raise YamlSubsetError(f'empty mapping for {key!r} is not supported')
            else:
                out.append(f'{pad}{k}: {_dump_scalar(value)}')
        return '\n'.join(out)
    if isinstance(data, list):
        for value in data:
            if isinstance(value, dict):
                body = dumps(value, indent + 2).splitlines()
                out.append(f'{pad}- {body[0].lstrip()}')
                out.extend(body[1:])
            elif isinstance(value, list):
                out.append(f'{pad}- [' + ', '.join(_dump_scalar(v) for v in value) + ']')
            else:
                out.append(f'{pad}- {_dump_scalar(value)}')
        return '\n'.join(out)
    return pad + _dump_scalar(data)


def dump(data: Any, path: Union[str, Path], header: str = '') -> None:
    text = (''.join(f'# {line}\n' for line in header.splitlines()) if header else '') + dumps(data) + '\n'
    Path(path).write_text(text)
