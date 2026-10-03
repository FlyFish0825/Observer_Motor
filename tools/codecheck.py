#!/usr/bin/env python3
"""Verify that an edit added only comments/whitespace, with zero code changes.

Usage:
    python codecheck.py <old_dir_or_file> <new_dir_or_file>

Compares the two versions after stripping comments, string-literal contents and
all whitespace, so line-number shifts and new comments are invisible while any
real code change shows up as a diff.  Exit code 0 = identical code.
"""
import sys
import difflib
from pathlib import Path

C_EXTS = {'.c', '.h', '.cpp', '.hpp', '.s', '.S'}


def tokens(text: str):
    """Return the code-only token stream: comments dropped, literals blanked."""
    i, n = 0, len(text)
    out = []
    while i < n:
        c = text[i]
        if c == '/' and i + 1 < n and text[i + 1] == '/':
            i += 2
            while i < n and text[i] != '\n':
                i += 1
            continue
        if c == '/' and i + 1 < n and text[i + 1] == '*':
            i += 2
            while i + 1 < n and not (text[i] == '*' and text[i + 1] == '/'):
                i += 1
            i += 2
            continue
        if c in '"\'':
            quote = c
            i += 1
            while i < n:
                if text[i] == '\\':
                    i += 2
                    continue
                if text[i] == quote:
                    i += 1
                    break
                i += 1
            out.append(quote + quote)
            continue
        if c.isspace():
            i += 1
            continue
        j = i
        while j < n:
            d = text[j]
            if d.isspace():
                break
            if d == '/' and j + 1 < n and text[j + 1] in '/*':
                break
            if d in '"\'':
                break
            j += 1
        if j > i:
            out.append(text[i:j])
            i = j
        else:
            out.append(c)
            i += 1
    return out


def normalize(path: Path):
    return tokens(path.read_text(encoding='utf-8', errors='replace'))


def cmp_file(old: Path, new: Path) -> bool:
    a, b = normalize(old), normalize(new)
    if a == b:
        return True
    print(f"CODE CHANGED: {new}")
    diff = list(difflib.unified_diff(a, b, lineterm='', n=3))
    for line in diff[:120]:
        print("   ", line)
    if len(diff) > 120:
        print(f"    ... {len(diff) - 120} more diff lines")
    return False


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    old_root, new_root = Path(sys.argv[1]), Path(sys.argv[2])
    pairs = []
    if old_root.is_file():
        if not new_root.is_file():
            print(f"MISSING: {new_root}")
            return 1
        pairs.append((old_root, new_root))
    else:
        for old in sorted(p for p in old_root.rglob('*') if p.is_file()):
            if old.suffix not in C_EXTS:
                continue
            pairs.append((old, new_root / old.relative_to(old_root)))
    bad = missing = 0
    for old, new in pairs:
        if not new.exists():
            print(f"MISSING: {new}")
            missing += 1
            continue
        if not cmp_file(old, new):
            bad += 1
    print(f"checked={len(pairs)} code_changed={bad} missing={missing}")
    return 1 if (bad or missing) else 0


if __name__ == '__main__':
    sys.exit(main())
