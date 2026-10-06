#!/usr/bin/env python3
"""Fail closed on missing documentation links and repository source citations."""
from __future__ import annotations
import argparse
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote

LINK = re.compile(r'!?\[[^\]]*\]\(([^)]+)\)')
INLINE_DOC = re.compile(r'`(docs/[A-Za-z0-9_./-]+)(?:#([^`]+))?`')
CITATION = re.compile(r'\b((?:src|tools|docs|config)/[A-Za-z0-9_./-]+):(\d+)(?:-(\d+))?')

def anchors(path: Path) -> set[str]:
    result = set()
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        if re.match(r'^#{1,6}\s', line):
            heading = re.sub(r'^#{1,6}\s+', '', line).strip().lower()
            result.add(re.sub(r'\s+', '-', re.sub(r'[^\w\s-]', '', heading)))
    return result

def check_document(root: Path, doc: Path) -> list[str]:
    errors = []
    for number, line in enumerate(doc.read_text(encoding='utf-8').splitlines(), 1):
        for match in LINK.finditer(line):
            target = match.group(1).strip().split(' "', 1)[0].strip('<>')
            if re.match(r'^[A-Za-z][A-Za-z0-9+.-]*:', target):
                continue
            name, _, fragment = unquote(target).partition('#')
            path = (doc.parent / name).resolve() if name else doc.resolve()
            if not path.exists():
                errors.append(f'{doc.relative_to(root)}:{number}: missing link {target}')
            elif fragment and path.suffix == '.md' and fragment not in anchors(path):
                errors.append(f'{doc.relative_to(root)}:{number}: missing anchor {target}')
        for match in INLINE_DOC.finditer(line):
            path = root / match.group(1)
            if not path.exists():
                errors.append(f'{doc.relative_to(root)}:{number}: missing document {match.group(1)}')
            elif match.group(2) and path.is_file() and match.group(2) not in anchors(path):
                errors.append(f'{doc.relative_to(root)}:{number}: missing anchor {match.group(0)}')
        for match in CITATION.finditer(line):
            path = root / match.group(1)
            if not path.is_file():
                errors.append(f'{doc.relative_to(root)}:{number}: missing citation {match.group(0)}')
            elif max(int(match.group(2)), int(match.group(3) or match.group(2))) > len(path.read_text(errors='replace').splitlines()):
                errors.append(f'{doc.relative_to(root)}:{number}: citation out of range {match.group(0)}')
    return errors

def tracked_documents(root: Path, directory: Path) -> list[Path]:
    """Return Markdown files under directory that git tracks or has not ignored.

    Untracked-but-not-ignored files count, so a new document is checked before it is
    committed; git-excluded files (e.g. the owner's private notes) are skipped.
    Falls back to rglob outside a git repo."""
    try:
        result = subprocess.run(
            ['git', '-C', str(root), 'ls-files', '-z', '--cached', '--others', '--exclude-standard',
             '--', str(directory.relative_to(root))],
            check=True, capture_output=True, text=True)
    except (OSError, subprocess.CalledProcessError, ValueError):
        return sorted(directory.rglob('*.md'))
    return sorted({root / name for name in result.stdout.split('\0')
                   if name.endswith('.md') and (root / name).is_file()})

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', nargs='?', default='.')
    parser.add_argument('docs', nargs='?', default='docs')
    args = parser.parse_args()
    root = Path(args.root).resolve()
    directory = (root / args.docs).resolve()
    if not directory.is_dir():
        print(f'documentation validation: FAIL (missing directory {args.docs})')
        return 1
    documents = tracked_documents(root, directory)
    if not documents:
        print('documentation validation: FAIL (no documentation)')
        return 1
    errors = [error for doc in documents for error in check_document(root, doc)]
    for doc in [root / 'README.md', root / 'AGENTS.md', *sorted((root / '.agents').rglob('*.md'))]:
        if doc.is_file():
            errors.extend(check_document(root, doc))
    # A retired design directory must not silently return as a second authority.
    if args.docs == 'docs':
        if (root / 'docs/refactor').exists():
            errors.append('retired docs/refactor directory still exists')
        if (root / 'docs/adr').exists():
            errors.append('retired docs/adr directory still exists')
        extra = [p for p in (root / 'docs/architecture').glob('*.md') if p.name not in ('SYSTEM_DESIGN.md', 'DECISIONS.md')]
        errors.extend(f'competing architecture document: {p.relative_to(root)}' for p in extra)
        for required in ['docs/architecture/SYSTEM_DESIGN.md', 'docs/architecture/DECISIONS.md', 'docs/TRACEABILITY.md']:
            if not (root / required).is_file():
                errors.append(f'missing canonical document: {required}')
    print('\n'.join(errors)) if errors else None
    print(f'documentation validation: {"FAIL" if errors else "PASS"} (documents={len(documents)}, errors={len(errors)})')
    return 1 if errors else 0

if __name__ == '__main__':
    raise SystemExit(main())
