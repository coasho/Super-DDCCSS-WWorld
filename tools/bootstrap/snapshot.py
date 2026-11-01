"""Copy the current source into a new empty directory, with content evidence.

This is a source export, not a Git worktree. Build products, environments,
process records, Git internals and every excluded directory are omitted.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

from project import inside, safe_member, sha256, write_json


SOURCE_PATHS = (
    "AGENTS.md", "CMakeLists.txt", "CMakePresets.json", "README.md", "LICENSE",
    "THIRD_PARTY_NOTICES.md", ".gitattributes", ".gitignore", ".gitmodules", ".github", ".claude/CLAUDE.md", "fsim.cmd", "Testbench.cmd",
    "setup.ps1", "build.ps1", "run.ps1", "stop.ps1", "include", "src", "python",
    "examples", "tests", "tools", "cmake", "deps", "third_party", "aircraft", "assets", "docs",
)


def snapshot(root, destination):
    root, destination = root.resolve(), destination.resolve()
    inside(root, destination.relative_to(root))
    if destination.exists() and any(destination.iterdir()):
        raise ValueError("Source export destination must be empty.")
    destination.mkdir(parents=True, exist_ok=True)
    command = ["git", "-C", str(root), "ls-files", "--cached", "--others", "--exclude-standard", "-z", "--", *SOURCE_PATHS,
               ":(exclude,glob)**/.codex/**"]
    names = subprocess.check_output(command).decode("utf-8").split("\0")
    sources = []
    for name in sorted(set(names)):
        if not name:
            continue
        if name.startswith(('tools/validation/missile/native/','tools/validation/missile/gpu_newton/','aircraft/aim120c/aero/')):
            continue  # Retired solver evidence is not a build input.
        safe_member(name)
        source = inside(root, name)
        if source.is_symlink():
            raise ValueError(f"Source export does not follow symlinks: {name}")
        if not source.is_file():
            continue  # A tracked deletion is absent from this working snapshot.
        sources.append((name, source, sha256(source)))
    # Git's checkout conversion is part of a real clone. Use an isolated
    # temporary index, never the user's index, branch or another worktree.
    temporary = inside(root, '.local/tmp')
    temporary.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=temporary) as folder:
        index = Path(folder)/'index'
        paths = Path(folder)/'paths'
        paths.write_bytes(b''.join(name.encode('utf-8')+b'\0' for name, _, _ in sources))
        env = dict(os.environ, GIT_INDEX_FILE=str(index))
        git = ['git', '--literal-pathspecs', '-C', str(root), '-c', 'core.safecrlf=false']
        subprocess.run([*git, 'read-tree', '--empty'], env=env, check=True)
        subprocess.run([*git, 'add', '--pathspec-from-file='+str(paths), '--pathspec-file-nul'],
                       env=env, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        subprocess.run([*git, 'checkout-index', '--all', '--force',
                        '--prefix='+destination.as_posix()+'/'], env=env, check=True)
    records = []
    for name, source, original in sources:
        target = inside(destination, name)
        if sha256(source) != original:
            raise ValueError(f'Source changed while exporting: {name}')
        digest = sha256(target)
        if digest != original:
            before, after = source.read_bytes(), target.read_bytes()
            if b'\0' in before or before.replace(b'\r\n', b'\n') != after.replace(b'\r\n', b'\n'):
                raise ValueError(f'Unexpected Git content conversion: {name}')
        records.append(dict(path=name, sha256=digest, source_sha256=original,
                            checkout_newlines_changed=digest != original, size=target.stat().st_size))
    head = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    record = {"schema": 2, "kind": "git-checkout-source-export", "source_head": head,
              "source_root": str(root), "project_root": str(destination), "file_count": len(records),
              "total_bytes": sum(x["size"] for x in records), "files": records}
    write_json(destination / ".local/state/source-snapshot.json", record)
    print(json.dumps({key: value for key, value in record.items() if key != "files"}, indent=2))
    return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", help="New empty directory inside the current project")
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[2]))
    args = parser.parse_args()
    snapshot(Path(args.root), Path(args.destination))
