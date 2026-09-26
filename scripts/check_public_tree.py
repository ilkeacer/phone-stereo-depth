"""Reject tracked content outside the audited text-only public source set."""
import subprocess
from pathlib import Path

from export_public import ROOT, public_files


def git(*args):
    return subprocess.run(
        ['git', *args], cwd=ROOT, check=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ).stdout


def main():
    allowed = {str(path.relative_to(ROOT)): path for path in public_files()}
    tracked = [name.decode('utf-8') for name in git('ls-files', '-z').split(b'\0') if name]
    outside = sorted(set(tracked) - set(allowed))
    if outside:
        raise SystemExit('Tracked files outside public allowlist: ' + ', '.join(outside))
    changed = []
    for name in tracked:
        staged = git('show', ':' + name)
        if staged != allowed[name].read_bytes():
            changed.append(name)
    if changed:
        raise SystemExit('Index differs from audited working tree: ' + ', '.join(changed))
    print(f'Public tree audit passed: {len(tracked)} tracked text files; no captured media or scene data.')


if __name__ == '__main__':
    main()
