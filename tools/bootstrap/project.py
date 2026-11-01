"""Project-contained Windows SDK, Viewer and Hangar orchestration.

No tool is installed globally. Dependency decisions come from the checked-in
lock, not from live package indexes. Job objects own every launched descendant.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import ctypes
from ctypes import wintypes
import hashlib
import importlib.metadata
import json
import ntpath
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import time
import tarfile
import tomllib
import urllib.request
import uuid
import zipfile

_BACKGROUND_CHILDREN = []


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def canonical(path):
    return ntpath.normcase(ntpath.normpath(str(path)))


def allowed_read_path(path):
    path = Path(path).absolute()
    if any(x.casefold() == ".codex" for x in path.parts):
        raise ValueError("Excluded directory in input path.")
    path = path.resolve()
    if any(x.casefold() == ".codex" for x in path.parts):
        raise ValueError("Excluded directory in input path.")
    return path


def inside(root, relative):
    root = allowed_read_path(root)
    candidate = root / relative
    if any(x.casefold() == ".codex" for x in candidate.parts):
        raise ValueError("Excluded directory in output path.")
    result = candidate.resolve()
    if not result.is_relative_to(root) or any(x.casefold() == ".codex" for x in result.parts):
        raise ValueError(f"Path leaves the project or enters an excluded directory: {relative}")
    return result


def safe_member(name):
    if "\\" in name or ":" in name or name.startswith("/"):
        raise ValueError(f"Unsafe archive member: {name}")
    path = PurePosixPath(name)
    if ".." in path.parts or any(x.casefold() == ".codex" for x in path.parts):
        raise ValueError(f"Unsafe archive member: {name}")
    return str(path)


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".new")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def load_lock(root):
    return json.loads(inside(root, "tools/bootstrap/dependencies.lock.json").read_text(encoding="utf-8"))


def validate_manifest(root, manifest, verify_tools=True):
    if manifest.get("schema") != 1 or canonical(manifest.get("project_root", "")) != canonical(root):
        raise ValueError("Manifest belongs to a different project path; rerun setup.ps1 after moving the checkout.")
    for name, tool in manifest["tools"].items():
        if not allowed_read_path(tool["path"]).is_file() or sha256(tool["path"]) != tool["sha256"]:
            raise ValueError(f"Resolved {name} changed or disappeared; rerun setup.ps1: {tool['path']}")


def load_manifest(root, verify_tools=True):
    path = inside(root, ".local/state/manifest.json")
    manifest = json.loads(path.read_text(encoding="utf-8"))
    validate_manifest(root, manifest, verify_tools)
    if manifest["lock_sha256"] != sha256(inside(root, "tools/bootstrap/dependencies.lock.json")):
        raise ValueError("Dependency lock changed; rerun setup.ps1.")
    return manifest


def project_environment(root, manifest, original=None):
    env = dict(os.environ if original is None else original)
    local = inside(root, ".local")
    for key, relative in {
        "TMP": "tmp", "TEMP": "tmp", "TORCH_EXTENSIONS_DIR": "tmp",
        "PIP_CACHE_DIR": "cache/pip", "MPLCONFIGDIR": "cache/matplotlib",
        "XDG_CACHE_HOME": "cache", "PYTHONPYCACHEPREFIX": "cache/pycache",
        "CCACHE_DIR": "cache/ccache", "UV_CACHE_DIR": "cache/uv",
    }.items():
        env[key] = "D:/ENV/tmp" if key in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR") else str(local / relative)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONNOUSERSITE"] = "1"
    env["PIP_DISABLE_PIP_VERSION_CHECK"] = "1"
    env["PIP_CONFIG_FILE"] = os.devnull
    env["FSIM_PROJECT_ROOT"] = str(root)
    env["FSIM_UCRT64_ROOT"] = Path(manifest["ucrt64_root"]).as_posix()
    env["FSIM_PYTHON_EXECUTABLE"] = manifest["tools"]["python"]["path"]
    env["FSIM_PRESET"] = "ucrt64-headless"
    env["FSIM_DLL_DIR"] = str(inside(root, "build/ucrt64-headless/bin"))
    env["HANGAR_MESHKIT_DIR"] = str(inside(root, "build/ucrt64-headless/python"))
    env["FSIM_AIRCRAFT_PATH"] = str(inside(root, "aircraft"))
    if "ninja" in manifest["tools"]:
        env["NINJA"] = manifest["tools"]["ninja"]["path"]
    env["MESON_PACKAGE_CACHE_DIR"] = str(local / "cache/meson")
    imports = manifest.get("python_packages", {}).get("imports", {})
    if imports.get("gmsh_module"):
        env["FSIM_HANGAR_GMSH_ROOT"] = str(Path(imports["gmsh_module"]).parent)
    paths = [str(Path(manifest["ucrt64_root"]) / "bin"), str(Path(manifest["tools"]["python"]["path"]).parent)]
    env["PATH"] = os.pathsep.join(paths + [env.get("PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([
        str(local / "packages"), str(inside(root, "tools/models/hangar")),
        str(inside(root, "build/ucrt64-headless/python")),
    ])
    env["FSIM_BOOTSTRAP_MANIFEST"] = str(local / "state/manifest.json")
    return env


def output(program, arguments=(), env=None):
    result = subprocess.run([str(program), *map(str, arguments)], text=True, capture_output=True,
                            env=env, timeout=60, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    if result.returncode:
        raise RuntimeError(f"Tool probe failed: {program}: {result.stderr.strip()}")
    return result.stdout.strip()


def fetch(root, item, offline=False):
    destination = inside(root, ".local/cache/archives/" + item["filename"])
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file() and sha256(destination) == item["sha256"]:
        return destination
    if offline:
        raise ValueError(f"Offline cache lacks pinned content: {item['filename']}")
    temporary = destination.with_suffix(destination.suffix + ".download")
    print(f"Download missing {item['filename']} ({item['size'] / 1048576:.1f} MiB)", flush=True)
    request = urllib.request.Request(item["url"], headers={"User-Agent": "flightsim-project-bootstrap/1"})
    with urllib.request.urlopen(request, timeout=90) as response, temporary.open("wb") as stream:
        shutil.copyfileobj(response, stream)
    if sha256(temporary) != item["sha256"]:
        raise ValueError(f"SHA256 mismatch for pinned archive: {item['filename']}")
    temporary.replace(destination)
    return destination


def install_sources(root, lock, offline):
    records = []
    for item in lock.get("source_archives", []):
        destination = inside(root, item["destination"])
        marker = destination / "bootstrap-source.json"
        if marker.is_file() and json.loads(marker.read_text())["archive_sha256"] == item["sha256"] and (destination / item["required"]).is_file():
            records.append({"name": item["name"], "path": str(destination), "sha256": item["sha256"]})
            continue
        archive = fetch(root, item, offline)
        destination.mkdir(parents=True, exist_ok=True)
        with tarfile.open(archive, "r:gz") as source:
            members = source.getmembers()
            for member in members:
                safe_member(member.name)
                if not member.isdir() and not member.isfile():
                    raise ValueError("Source archive contains links or special files.")
                relative = PurePosixPath(member.name).relative_to(item["archive_root"])
                target = inside(root, str(destination.relative_to(root) / str(relative)))
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with source.extractfile(member) as stream, target.open("wb") as out:
                        shutil.copyfileobj(stream, out)
        write_json(marker, {"archive_sha256": item["sha256"], "upstream": item["upstream"]})
        records.append({"name": item["name"], "path": str(destination), "sha256": item["sha256"]})
    # Some upstream source filenames match their own broad ignore patterns.
    # Provision these exact required text files when a source export omits them.
    for item in lock.get("source_files", []):
        destination = inside(root, item["destination"])
        approved = {item["sha256"], *item.get("accepted_sha256", [])}
        if destination.exists():
            if not destination.is_file() or sha256(destination) not in approved:
                raise ValueError("Pinned vendor source changed; preserve and review: " + str(destination))
        else:
            source = fetch(root, item, offline)
            if sha256(source) != item["sha256"]:
                raise ValueError("Pinned vendor source download changed: " + str(source))
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            if sha256(destination) != item["sha256"]:
                raise ValueError("Pinned vendor source copy changed: " + str(destination))
        records.append({"name": item["name"], "path": str(destination), "sha256": sha256(destination)})
    return records


def install_geometry(root, lock, offline):
    """Pinned meshing headers, missing-only in the project-owned prefix."""
    records=[]
    for item in lock.get('geometry_archives',[]):
        destination=inside(root,item['destination']);marker=destination/'bootstrap-geometry.json'
        previous=json.loads(marker.read_text(encoding='utf-8')) if marker.is_file() else {}
        files=previous.get('files',[])
        if (previous.get('archive_sha256')==item['sha256'] and files
                and all(inside(root,str(destination.relative_to(root)/safe_member(name))).is_file() for name in files)):
            records.append(dict(name=item['name'],path=str(destination),sha256=item['sha256']));continue
        archive=fetch(root,item,offline);files=[]
        context=native_package_archive(root,archive) if item['format']=='zstd' else tarfile.open(archive,'r:*')
        with context as source:
            for member in source:
                name=safe_member(member.name)
                if item['archive_root']:
                    name=str(PurePosixPath(name).relative_to(item['archive_root']))
                if name=='.':continue
                if not member.isdir() and not member.isfile():raise ValueError('Geometry archive contains links or special files')
                target=inside(root,str(destination.relative_to(root)/safe_member(name)))
                if member.isdir():target.mkdir(parents=True,exist_ok=True)
                else:
                    files.append(name);target.parent.mkdir(parents=True,exist_ok=True)
                    if not target.is_file():
                        with source.extractfile(member) as stream,target.open('xb') as out:shutil.copyfileobj(stream,out)
        if not files or not (destination/item['required']).is_file():raise ValueError('Incomplete pinned geometry archive')
        write_json(marker,dict(archive_sha256=item['sha256'],files=files))
        records.append(dict(name=item['name'],path=str(destination),sha256=item['sha256']))
    return records


@contextmanager
def native_package_archive(root, archive):
    sys.path.insert(0, str(inside(root, ".local/packages")))
    try:
        import zstandard
    finally:
        sys.path.pop(0)
    temporary = inside(root, ".local/tmp/native-package-" + uuid.uuid4().hex + ".tar")
    temporary.parent.mkdir(parents=True, exist_ok=True)
    try:
        with Path(archive).open("rb") as source, temporary.open("wb") as target:
            zstandard.ZstdDecompressor().copy_stream(source, target)
        with tarfile.open(temporary, "r:") as source:
            yield source
    finally:
        if temporary.exists():
            temporary.unlink()


def native_payload_files(files):
    """Canonical file inventory; directories and package metadata are separate."""
    if not isinstance(files, list) or not files:
        raise ValueError("Nonempty native package payload inventory required.")
    result = []
    seen = set()
    for name in files:
        if not isinstance(name, str):
            raise ValueError("Native payload inventory paths must be strings.")
        name = safe_member(name)
        if not name.startswith("ucrt64/") or name.endswith("/") or name.casefold() in seen:
            raise ValueError("Invalid or duplicate native payload inventory path: " + name)
        seen.add(name.casefold())
        result.append(name)
    return sorted(result)


def native_payload_target(root, destination, name):
    destination = inside(root, Path(destination).relative_to(root))
    lexical = destination / name
    for part in (lexical, *lexical.parents):
        if part == destination.parent:
            break
        if part.is_symlink() or part.is_junction():
            raise ValueError("Native payload does not follow existing links: " + str(part))
        if part != lexical and part.exists() and not part.is_dir():
            raise ValueError("Native payload parent conflicts with an existing file: " + str(part))
    target = inside(root, lexical.relative_to(root))
    if not target.is_relative_to(destination / "ucrt64"):
        raise ValueError("Native payload leaves its install prefix.")
    return target


def native_archive_members(root, source, destination):
    """Validate the entire archive and link graph before any payload write."""
    regular, links, seen = {}, {}, set()
    metadata = {".PKGINFO", ".BUILDINFO", ".MTREE", ".INSTALL", ".CHANGELOG"}
    for member in source.getmembers():
        name = safe_member(member.name)
        if name.casefold() in seen:
            raise ValueError("Duplicate native archive member: " + name)
        seen.add(name.casefold())
        if not (member.isdir() or member.isfile() or member.issym() or member.islnk()):
            raise ValueError("Native package contains a device or special record.")
        if member.isdir():
            continue
        if name in metadata and member.isfile():
            continue
        native_payload_files([name])
        target = native_payload_target(root, destination, name)
        if member.isfile():
            regular[name] = member
        else:
            linkname = member.linkname
            if (not linkname or "\\" in linkname or ":" in linkname or PurePosixPath(linkname).is_absolute()):
                raise ValueError("Unsafe native package link target.")
            base = target.parent if member.issym() else destination
            resolved = inside(root, base / linkname)
            if not resolved.is_relative_to(destination / "ucrt64"):
                raise ValueError("Native package link leaves its install prefix.")
            relative = resolved.relative_to(destination).as_posix()
            native_payload_target(root, destination, relative)
            links[name] = relative
    names = native_payload_files(list(regular) + list(links))
    canonical = {name.casefold(): name for name in names}
    for name in names:
        if any(parent.as_posix().casefold() in canonical for parent in PurePosixPath(name).parents):
            raise ValueError("Native archive file/link is another payload path's parent: " + name)
    resolved_links = {}
    def resolve(name, visiting):
        name = canonical.get(name.casefold(), name)
        if name in regular:
            return ("archive", regular[name])
        if name in visiting:
            raise ValueError("Native package has unresolved or cyclic links.")
        if name in links:
            return resolve(links[name], visiting | {name})
        existing = native_payload_target(root, destination, name)
        if existing.is_file():
            return ("existing", existing)
        raise ValueError("Native package has unresolved or cyclic links.")
    for name in links:
        resolved_links[name] = resolve(name, set())
    return names, regular, resolved_links


def missing_native_payload(root, destination, files):
    missing = []
    for name in native_payload_files(files):
        target = native_payload_target(root, destination, name)
        if target.exists() and not target.is_file():
            raise ValueError("Native payload file conflicts with an existing directory: " + str(target))
        if not target.is_file():
            missing.append(name)
    return missing


def extract_native_package(root, archive, destination, *, missing=None, expected_files=None):
    destination = inside(root, Path(destination).relative_to(root))
    with native_package_archive(root, archive) as source:
        files, regular, links = native_archive_members(root, source, destination)
        if expected_files is not None and files != native_payload_files(expected_files):
            raise ValueError("Native package repair inventory differs from its pinned archive.")
        wanted = files if missing is None else native_payload_files(list(missing))
        if not set(wanted) <= set(files):
            raise ValueError("Repair members differ from the pinned native archive payload.")
        wanted = missing_native_payload(root, destination, wanted)
        for name in wanted:
            target = native_payload_target(root, destination, name)
            kind, value = ("archive", regular[name]) if name in regular else links[name]
            target.parent.mkdir(parents=True, exist_ok=True)
            # Existing payload files are never overwritten, including link copies.
            with (source.extractfile(value) if kind == "archive" else value.open("rb")) as stream, target.open("xb") as out:
                shutil.copyfileobj(stream, out)
    return files


def read_pacman_fields(path):
    text = Path(path).read_text(encoding="utf-8")
    return {key: value.strip().splitlines() for key, value in
            re.findall(r"%([A-Z0-9]+)%\n(.*?)(?=\n%[A-Z0-9]+%\n|\Z)", text, re.S)}


def installed_native(ucrt64):
    """Read package metadata only; never run pacman against an existing tree."""
    database = Path(ucrt64).parent / "var/lib/pacman/local"
    if not database.is_dir():
        return {}
    rows = {}
    for entry in database.iterdir():
        if entry.name.casefold() == ".codex" or not entry.is_dir():
            continue
        description = entry / "desc"
        if not description.is_file():
            continue
        data = read_pacman_fields(description)
        if data.get("NAME") and data.get("VERSION"):
            rows[data["NAME"][0]] = {"version": data["VERSION"][0], "directory": str(entry)}
    return rows


def native_complete(ucrt64, lock):
    rows = installed_native(ucrt64)
    for item in lock["native_packages"]:
        row = rows.get(item["name"], {})
        files = installed_native_files(row)
        if (row.get("version") != item["version"] or files is None
                or not complete_native_reference(ucrt64, files)):
            return False
    return bool(rows)


def installed_native_files(installed):
    directory = installed.get("directory")
    if not directory:
        return None
    path = allowed_read_path(Path(directory) / "files")
    if not path.is_file():
        return None
    records = read_pacman_fields(path).get("FILES")
    if records is None:
        return None
    files = []
    for name in records:
        canonical_name = safe_member(name)
        if canonical_name.startswith("ucrt64/") and not name.endswith("/"):
            files.append(canonical_name)
    return native_payload_files(files) if files else None


def complete_native_reference(ucrt64, files):
    prefix = allowed_read_path(ucrt64)
    for name in files:
        source = prefix.parent / name
        if source.is_symlink() or source.is_junction() or not source.is_file():
            return False
        if not allowed_read_path(source).is_relative_to(prefix):
            return False
    return True


def native_candidates(explicit=None):
    candidates = [explicit, os.environ.get("FSIM_UCRT64_ROOT")]
    compiler = shutil.which("gcc.exe")
    if compiler:
        candidates.append(str(Path(compiler).parent.parent))
    candidates.append("D:/ENV/DevLanguages/Cpp/msys2/ucrt64")
    return list(dict.fromkeys(str(allowed_read_path(x)) for x in candidates if x))


def copy_installed_package(root, reference, installed, item, destination, *, missing=None):
    files = installed_native_files(installed)
    if files is None or not complete_native_reference(reference, files):
        return False
    source_root = Path(reference).parent
    wanted = files if missing is None else native_payload_files(list(missing))
    if not set(wanted) <= set(files):
        return False
    wanted = missing_native_payload(root, destination, wanted)
    for name in wanted:
        target = native_payload_target(root, destination, name)
        target.parent.mkdir(parents=True, exist_ok=True)
        with (source_root / name).open("rb") as stream, target.open("xb") as out:
            shutil.copyfileobj(stream, out)
    return True


def install_native(root, lock, explicit, offline):
    local_root = inside(root, ".local/msys2")
    installed_record = local_root / "bootstrap-packages.json"
    lock_sha = sha256(inside(root, "tools/bootstrap/dependencies.lock.json"))
    record = json.loads(installed_record.read_text(encoding="utf-8")) if installed_record.is_file() else None
    inventory = {}
    if record is not None:
        if (not isinstance(record, dict) or record.get("schema") not in (1, 2)
                or not isinstance(record.get("lock_sha256"), str)
                or not re.fullmatch(r"[0-9a-f]{64}", record["lock_sha256"])
                or (record["schema"] == 1 and record["lock_sha256"] != lock_sha)):
            raise ValueError("Existing native package record differs from this lock; preserve its prefix.")
        packages = record.get("packages")
        if not isinstance(packages, list) or any(not isinstance(row, dict) for row in packages):
            raise ValueError("Invalid native package record.")
        previous = {row.get("name"): row for row in packages}
        if len(previous) != len(packages):
            raise ValueError("Duplicate native package record.")
        if set(previous) != {item["name"] for item in lock["native_packages"]}:
            raise ValueError("Native package record does not contain the complete locked roster.")
        if any(previous[item["name"]].get("version") != item["version"] for item in lock["native_packages"]):
            raise ValueError("Native package record versions differ from the pinned lock.")
        if record["schema"] == 2:
            for item in lock["native_packages"]:
                row = previous.get(item["name"], {})
                if row.get("version") != item["version"] or row.get("archive_sha256") != item["sha256"]:
                    raise ValueError("Native package inventory differs from the pinned lock: " + item["name"])
                inventory[item["name"]] = native_payload_files(row.get("files"))
            if all(not missing_native_payload(root, local_root, files) for files in inventory.values()):
                # Schema2 binds each native archive and its complete payload inventory.
                # Unrelated source-file pins may evolve without reinstalling that prefix.
                if record["lock_sha256"] != lock_sha:
                    write_json(installed_record, dict(record, lock_sha256=lock_sha))
                return local_root / "ucrt64", {"mode": "project-local", "packages": str(installed_record)}
    candidates = native_candidates(explicit)
    if record is None:
        for candidate in candidates:
            if native_complete(candidate, lock):
                return Path(candidate), {"mode": "existing-exact-package-versions", "package_root": candidate}
    # Missing packages are downloaded. Exact installed packages can be copied
    # into the local prefix, without altering the shared environment.
    references = [(x, installed_native(x)) for x in candidates]
    local_root.mkdir(parents=True, exist_ok=True)
    decisions = []
    for item in lock["native_packages"]:
        archive = None
        files = inventory.get(item["name"])
        if files is None:
            for reference, rows in references:
                row = rows.get(item["name"], {})
                if row.get("version") == item["version"]:
                    files = installed_native_files(row)
                    if files is not None:
                        break
            if files is None:
                try:
                    archive = fetch(root, item, offline)
                except ValueError as error:
                    raise ValueError("Native payload inventory unavailable for " + item["name"] + ": " + str(error)) from error
                with native_package_archive(root, archive) as source:
                    files, _, _ = native_archive_members(root, source, local_root)
            inventory[item["name"]] = files
        missing = missing_native_payload(root, local_root, files)
        decision = {"name": item["name"], "version": item["version"], "archive_sha256": item["sha256"], "files": files}
        if not missing:
            decisions.append(dict(decision, action="reused-complete"))
            continue
        copied = False
        for reference, rows in references:
            row = rows.get(item["name"], {})
            if (row.get("version") == item["version"] and installed_native_files(row) == files
                    and copy_installed_package(root, reference, row, item, local_root, missing=missing)):
                decisions.append(dict(decision, source=reference, action="copied-missing", repaired_files=missing))
                copied = True
                break
        if copied:
            continue
        archive = archive or fetch(root, item, offline)
        extract_native_package(root, archive, local_root, missing=missing, expected_files=files)
        decisions.append(dict(decision, action="extracted-missing-pinned", repaired_files=missing))
    for item in lock["native_packages"]:
        if missing_native_payload(root, local_root, inventory[item["name"]]):
            raise ValueError("Native package repair is incomplete: " + item["name"])
    write_json(installed_record, {"schema": 2, "lock_sha256": lock_sha, "packages": decisions})
    return local_root / "ucrt64", {"mode": "project-local", "packages": str(installed_record)}


def package_versions(root):
    code = "import importlib.metadata as m,json; v={}; [v.setdefault(d.metadata['Name'].lower().replace('_','-'),d.version) for d in m.distributions()]; print(json.dumps(v))"
    env = dict(os.environ, PYTHONPATH=str(inside(root, ".local/packages")), PYTHONNOUSERSITE="1")
    return json.loads(output(sys.executable, ["-B", "-c", code], env))


def install_python_packages(root, lock, offline):
    versions = package_versions(root)
    missing = [x for x in lock["python_packages"] if versions.get(x["name"].lower().replace("_", "-")) != x["version"]]
    packages = inside(root, ".local/packages")
    packages.mkdir(parents=True, exist_ok=True)
    # pip itself is a pinned pure-Python wheel; run it from the verified wheel
    # when the NuGet CI interpreter does not already provide pip.
    pip_record = next(x for x in lock["python_packages"] if x["name"] == "pip")
    if missing:
        pip_wheel = fetch(root, pip_record, offline)
        archives = [fetch(root, item, offline) for item in missing]
        env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(pip_wheel), str(packages)]), PYTHONNOUSERSITE="1")
        command = [sys.executable, "-B", "-m", "pip", "--isolated", "--disable-pip-version-check", "--cache-dir", str(inside(root, ".local/cache/pip")), "install", "--no-index", "--no-deps", "--no-compile", "--upgrade", "--target", str(packages), *map(str, archives)]
        subprocess.run(command, cwd=root, env=env, check=True)
    # pip --target can omit a wheel's data/lib DLL. Preserve Gmsh's native
    # payload beside its Python API, which its documented loader searches.
    if (any(item['name'] == 'gmsh' for item in lock['python_packages']) and
            (packages / "gmsh.py").is_file() and not (packages / "gmsh-4.15.dll").is_file()):
        gmsh_record = next(x for x in lock["python_packages"] if x["name"] == "gmsh")
        install_gmsh_library(root, fetch(root, gmsh_record, offline))
    versions = package_versions(root)
    for item in lock["python_packages"]:
        if versions.get(item["name"].lower().replace("_", "-")) != item["version"]:
            raise ValueError(f"Pinned Python package probe failed: {item['name']}")
    return {"mode": "missing-only-project-overlay", "versions": versions, "installed": [x["name"] for x in missing]}


def install_gmsh_library(root, archive):
    with zipfile.ZipFile(archive) as wheel:
        entries = [name for name in wheel.namelist() if re.fullmatch(r"gmsh-[^/]+\.data/data/lib/gmsh-[0-9.]+\.dll", name)]
        if len(entries) != 1:
            raise ValueError("Pinned Gmsh wheel must contain exactly one native library.")
        name = safe_member(entries[0])
        destination = inside(root, ".local/packages/" + PurePosixPath(name).name)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with wheel.open(name) as source, destination.open("wb") as target:
            shutil.copyfileobj(source, target)
        return destination


def probe_python_packages(root):
    env = dict(os.environ, PYTHONPATH=str(inside(root, ".local/packages")), PYTHONNOUSERSITE="1")
    code = "import numpy,scipy,matplotlib,PIL,json; print(json.dumps({'numpy':numpy.__version__,'scipy':scipy.__version__,'matplotlib':matplotlib.__version__,'Pillow':PIL.__version__}))"
    return json.loads(output(sys.executable, ["-B", "-c", code], env))


def setup(root, args):
    if os.name != "nt":
        raise ValueError("This locked bootstrap supports Windows x64 UCRT64 only.")
    lock = load_lock(root)
    if ".".join(map(str, sys.version_info[:3])) != lock["python"]["version"] or sys.maxsize <= 2**32:
        raise ValueError("Python does not match the pinned x64 interpreter.")
    base_manifest = {"ucrt64_root": str(inside(root, ".local/msys2/ucrt64")), "tools": {"python": {"path": sys.executable}}}
    os.environ.update(project_environment(root, base_manifest))
    python_packages = install_python_packages(root, lock, args.offline)
    ucrt64, provenance = install_native(root, lock, args.ucrt64_root, args.offline)
    python_packages["imports"] = probe_python_packages(root)
    sources = install_sources(root, lock, args.offline)
    runtime_assets = [{"name": item["name"], "path": str(fetch(root, item, args.offline)), "sha256": item["sha256"]} for item in lock.get("runtime_assets", [])]
    tools = {"python": {"path": str(Path(sys.executable).resolve()), "version": lock["python"]["version"]}}
    for name, arguments in (("gcc", ["-dumpfullversion"]), ("g++", ["-dumpfullversion"]), ("cmake", ["--version"]), ("ninja", ["--version"]), ("ctest", ["--version"])):
        executable = ucrt64 / "bin" / (name + ".exe")
        tools[name] = {"path": str(executable), "version": output(executable, arguments).splitlines()[0]}
    for tool in tools.values():
        tool["sha256"] = sha256(tool["path"])
    manifest = {"schema": 1, "project_root": str(root), "lock_sha256": sha256(inside(root, "tools/bootstrap/dependencies.lock.json")),
                "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "tools": tools, "ucrt64_root": str(ucrt64),
                "native_provenance": provenance, "python_packages": python_packages, "source_archives": sources, "runtime_assets": runtime_assets, 
                "method": "vlm", "pending": []}
    write_json(inside(root, ".local/state/manifest.json"), manifest)
    print(json.dumps({"setup": "passed", "manifest": str(inside(root, ".local/state/manifest.json")), "pending": manifest["pending"]}), flush=True)
    return 0


def discover_models(root):
    directory = inside(root, "aircraft")
    return sorted(entry.name for entry in directory.iterdir()
                  if entry.name.casefold() != ".codex" and entry.is_dir() and (entry / (entry.name + ".toml")).is_file())


def select_models(root, requested):
    available = discover_models(root)
    selected = []
    for name in requested or ["all"]:
        if name == "all":
            selected.extend(available)
        elif name == "stores":
            for model in available:
                data = tomllib.loads(inside(root, f"aircraft/{model}/{model}.toml").read_text(encoding="utf-8"))
                if data.get("aircraft", {}).get("card"):
                    selected.append(model)
        elif name in available:
            selected.append(name)
        else:
            raise ValueError(f"Unknown Hangar model: {name}")
    return list(dict.fromkeys(selected))


def configure_arguments(root, manifest, preset):
    render = "ON" if preset == "ucrt64-release" else "OFF"
    return ["-S", str(root), "-B", str(inside(root, f"build/{preset}")), "-G", "Ninja",
            "-DCMAKE_TOOLCHAIN_FILE=" + inside(root, "cmake/toolchains/ucrt64.cmake").as_posix(),
            "-DFSIM_UCRT64_ROOT=" + Path(manifest["ucrt64_root"]).as_posix(),
            "-DCMAKE_MAKE_PROGRAM=" + Path(manifest["tools"]["ninja"]["path"]).as_posix(),
            "-DCMAKE_BUILD_TYPE=Release", "-DFSIM_BUILD_PYTHON=ON", "-DFSIM_BUILD_TESTS=ON",
            "-DFSIM_WITH_RENDER=" + render,
            "-DFSIM_PYTHON_EXECUTABLE=" + Path(manifest["tools"]["python"]["path"]).as_posix(),
            "-DFSIM_PYTHON_TEST_PATH=" + inside(root, ".local/packages").as_posix(),
            "-DCMAKE_PREFIX_PATH=" + inside(root, "build/deps-ucrt64/install").as_posix(),
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF", "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF",
            "-DFETCHCONTENT_BASE_DIR=" + inside(root, ".local/cache/fetchcontent/" + preset).as_posix(),
            "-DFETCHCONTENT_SOURCE_DIR_CATCH2=" + inside(root, ".local/cache/sources/catch2-3.7.1").as_posix()]


def build_commands(root, manifest, args):
    cmake = manifest["tools"]["cmake"]["path"]
    deps = inside(root, "build/deps-ucrt64")
    commands = []
    commands += [[manifest["tools"]["python"]["path"], "-B", str(inside(root, "tools/bootstrap/project.py")), "--root", str(root), "_stage-assets"],
                [cmake, "-S", str(inside(root, "deps")), "-B", str(deps), "-G", "Ninja",
                 "-DCMAKE_TOOLCHAIN_FILE=" + inside(root, "cmake/toolchains/ucrt64.cmake").as_posix(),
                 "-DFSIM_UCRT64_ROOT=" + Path(manifest["ucrt64_root"]).as_posix(),
                 "-DCMAKE_MAKE_PROGRAM=" + Path(manifest["tools"]["ninja"]["path"]).as_posix(),
                 "-DFSIM_DEPS_INSTALL_PREFIX=" + (deps / "install").as_posix()],
                [cmake, "--build", str(deps), "--parallel", str(args.jobs)],
                [cmake, *configure_arguments(root, manifest, "ucrt64-headless")],
                [cmake, "--build", str(inside(root, "build/ucrt64-headless")), "--target", "hangar_meshkit", "--parallel", str(args.jobs)]]
    models = select_models(root, args.model)
    if not models:
        raise ValueError("The requested Hangar model roster is empty.")
    output = inside(root, ".local/campaigns/hangar/" + uuid.uuid4().hex)
    if output.exists():
        raise ValueError("Preserve the existing Hangar campaign: " + str(output))
    command = [manifest["tools"]["python"]["path"], "-B", "-m", "hangar.fleet_build",
               "--jobs", str(args.jobs),
               "--memory-limit-mib", str(getattr(args, "memory_limit_mib", 4096)),
               "--out", str(output)]
    command += ["--runtime-threads", "1", "--runtime-memory-mib", "512"]
    for model in models:
        command += ["--model", model]
    # A normal product build regenerates aerodynamic data and XML. Expensive
    # mesh/performance campaigns remain explicit Hangar stages.
    position=command.index('--out')+1
    aerodynamic=list(command);aerodynamic[position]=str(output/'aero')
    aerodynamic += ['--stage','aero']
    calibration=[manifest['tools']['python']['path'],'-B','-m','hangar.calibration','--out',str(output/'calibration.json')]
    for model in models:calibration += ['--model',model]
    publication=list(command);publication[position]=str(output/'build')
    publication += ['--stage','build']
    commands.extend([aerodynamic,calibration,publication])
    # Re-stage generated model and XML outputs after Hangar completed.
    for preset in ("ucrt64-headless", "ucrt64-release"):
        commands += [[cmake, *configure_arguments(root, manifest, preset)],
                     [cmake, "--build", str(inside(root, f"build/{preset}")), "--parallel", str(args.jobs)]]
        if args.test:
            commands.append([cmake, "-E", "env", "FSIM_PRESET="+preset,
                             "FSIM_DLL_DIR="+str(inside(root, f"build/{preset}/bin")),
                             manifest["tools"]["ctest"]["path"], "--test-dir", str(inside(root, f"build/{preset}")),
                             "--output-on-failure", "--no-tests=error", "--timeout", "1200", "--parallel", str(args.jobs)])
    return commands


def run_commands(root, manifest, args):
    build = inside(root, "build/" + args.preset)
    if args.mode in ("viewer", "demo"):
        if args.preset != "ucrt64-release":
            raise ValueError("Viewer/demo require ucrt64-release.")
        command = [str(build / "bin/flightsim-viewer.exe")]
        if args.mode == "demo":
            command += ["--demo", "--vehicles", "6"]
    elif args.mode == "headless":
        command = [str(build / "bin/flightsim.exe")]
    else:
        command = [manifest["tools"]["python"]["path"], "-B"]
        if args.mode == "hangar":
            command += ["-m", "hangar"]
    if not Path(command[0]).is_file():
        raise ValueError(f"Executable is missing; run build.ps1: {command[0]}")
    if args.mode == "hangar":
        # Refresh the shared geometry library before a direct Hangar run.
        commands = [[manifest["tools"]["cmake"]["path"], "--build", str(build),
                 "--target", "hangar_meshkit", "--parallel", "2"],
                command + args.arguments]
        return commands
    return [command + args.arguments]


def identity_matches(record, live, root):
    return bool(live and record.get("pid") == live.get("pid") and record.get("created") == live.get("created")
                and canonical(record.get("executable", "")) == canonical(live.get("executable", ""))
                and canonical(record.get("root", "")) == canonical(root)
                and re.fullmatch(r"[0-9a-f]{32}", record.get("token", "")))


class _JobBasicLimits(ctypes.Structure):
    _fields_ = [("PerProcessUserTimeLimit", ctypes.c_longlong), ("PerJobUserTimeLimit", ctypes.c_longlong),
                ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD), ("SchedulingClass", wintypes.DWORD)]


class _IoCounters(ctypes.Structure):
    _fields_ = [(name, ctypes.c_ulonglong) for name in
                ("ReadOperationCount", "WriteOperationCount", "OtherOperationCount",
                 "ReadTransferCount", "WriteTransferCount", "OtherTransferCount")]


class _JobExtendedLimits(ctypes.Structure):
    _fields_ = [("BasicLimitInformation", _JobBasicLimits), ("IoInfo", _IoCounters),
                ("ProcessMemoryLimit", ctypes.c_size_t), ("JobMemoryLimit", ctypes.c_size_t),
                ("PeakProcessMemoryUsed", ctypes.c_size_t), ("PeakJobMemoryUsed", ctypes.c_size_t)]


class _JobAccounting(ctypes.Structure):
    _fields_ = [(name, ctypes.c_longlong) for name in
                ("TotalUserTime", "TotalKernelTime", "ThisPeriodTotalUserTime", "ThisPeriodTotalKernelTime")] + [
                (name, wintypes.DWORD) for name in
                ("TotalPageFaultCount", "TotalProcesses", "ActiveProcesses", "TotalTerminatedProcesses")]


def windows_api():
    if os.name != "nt":
        raise ValueError("Process ownership requires Windows job objects.")
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    api.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    api.OpenProcess.restype = wintypes.HANDLE
    api.GetCurrentProcess.restype = wintypes.HANDLE
    api.CloseHandle.argtypes = [wintypes.HANDLE]
    api.GetProcessTimes.argtypes = [wintypes.HANDLE, *([ctypes.POINTER(wintypes.FILETIME)] * 4)]
    api.QueryFullProcessImageNameW.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD)]
    api.GetExitCodeProcess.argtypes = [wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)]
    api.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    api.CreateJobObjectW.restype = wintypes.HANDLE
    api.OpenJobObjectW.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.LPCWSTR]
    api.OpenJobObjectW.restype = wintypes.HANDLE
    api.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    api.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    api.SetInformationJobObject.restype = wintypes.BOOL
    api.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
    api.QueryInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p,
                                            wintypes.DWORD, ctypes.c_void_p]
    api.QueryInformationJobObject.restype = wintypes.BOOL
    api.IsProcessInJob.argtypes = [wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)]
    return api


def _wait_job_empty(api, job, timeout=10):
    # TerminateJobObject initiates asynchronous termination. Do not report
    # completion while descendants can still retain output files or the slot.
    deadline = time.monotonic() + timeout
    while True:
        accounting = _JobAccounting()
        if not api.QueryInformationJobObject(job, 1, ctypes.byref(accounting), ctypes.sizeof(accounting), None):
            raise OSError(ctypes.get_last_error(), "Cannot verify owned project job termination.")
        if accounting.ActiveProcesses == 0:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Owned project job still has active processes after termination.")
        time.sleep(0.02)


def process_identity(pid):
    api = windows_api()
    handle = api.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
    if not handle:
        return None
    try:
        creation, exit_time, kernel, user = (wintypes.FILETIME() for _ in range(4))
        if not api.GetProcessTimes(handle, ctypes.byref(creation), ctypes.byref(exit_time), ctypes.byref(kernel), ctypes.byref(user)):
            return None
        size = wintypes.DWORD(32768)
        image = ctypes.create_unicode_buffer(size.value)
        if not api.QueryFullProcessImageNameW(handle, 0, image, ctypes.byref(size)):
            return None
        return {"pid": pid, "created": (creation.dwHighDateTime << 32) | creation.dwLowDateTime, "executable": image.value}
    finally:
        api.CloseHandle(handle)


def process_running(pid):
    api = windows_api()
    handle = api.OpenProcess(0x1000, False, pid)
    if not handle:
        return False
    try:
        status = wintypes.DWORD()
        return bool(api.GetExitCodeProcess(handle, ctypes.byref(status)) and status.value == 259)
    finally:
        api.CloseHandle(handle)


def job_name(root, token):
    return "Local\\fsim-" + hashlib.sha256(canonical(root).encode()).hexdigest()[:20] + "-" + token


def worker(root, plan):
    document = json.loads(inside(root, plan).read_text(encoding="utf-8"))
    token = document["token"]
    if not re.fullmatch(r"[0-9a-f]{32}", token):
        raise ValueError("Invalid launch token.")
    api = windows_api()
    name = job_name(root, token)
    job = api.CreateJobObjectW(None, name)
    if not job:
        raise OSError(ctypes.get_last_error(), "Cannot establish an isolated project process job.")
    limits = _JobExtendedLimits()
    limits.BasicLimitInformation.LimitFlags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    # The worker owns the only job handle. A crash or forced worker termination
    # must reap descendants even when Python's finally block cannot execute.
    if (not api.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits))
            or not api.AssignProcessToJobObject(job, api.GetCurrentProcess())):
        error = ctypes.get_last_error()
        api.CloseHandle(job)
        raise OSError(error, "Cannot establish crash-safe project process ownership.")
    record = dict(process_identity(os.getpid()), root=str(root), token=token, job=name, state="running", plan=str(plan))
    record_path = inside(root, ".local/state/processes/" + token + ".json")
    write_json(record_path, record)
    status, command_results, slot = 1, [], None
    try:
        if document.get("lock"):
            import msvcrt
            lock_path = inside(root, ".local/state/" + document["lock"] + ".lock")
            slot = lock_path.open("a+b")
            if slot.tell() == 0:
                slot.write(b"0")
                slot.flush()
            slot.seek(0)
            try:
                msvcrt.locking(slot.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError as error:
                raise RuntimeError(f"Project {document['lock']} slot is already held; wait or use stop.ps1.") from error
        env = dict(os.environ, **document["environment"])
        for index, command in enumerate(document["commands"]):
            if document["kind"] == "build":
                env = project_environment(root, load_manifest(root, verify_tools=False), env)
            started = time.monotonic()
            log = inside(root, f".local/logs/{token}-{index:03d}.log")
            print(f"[{index + 1}/{len(document['commands'])}] {subprocess.list2cmdline(command)}\n  log: {log}", flush=True)
            with log.open("w", encoding="utf-8") as stream:
                child = subprocess.Popen(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                         creationflags=subprocess.CREATE_NO_WINDOW)
                status = child.wait()
            command_results.append({"command": command, "exit_code": status, "elapsed_seconds": time.monotonic() - started, "log": str(log)})
            if status:
                print(f"Command failed ({status}); raw output: {log}", flush=True)
                break
        if status == 0 and document["kind"] == "build":
            load_manifest(root)  # Recheck all proof inputs once after the immutable build window.
        record.update(state="completed" if status == 0 else "failed", exit_code=status, results=command_results)
        write_json(record_path, record)
        write_json(inside(root, ".local/state/last-" + document["kind"] + ".json"), record)
        print(f"Project {document['kind']} {'completed' if status == 0 else 'failed'}; evidence: {record_path}", flush=True)
        return status
    except BaseException as error:
        status = 1
        record.update(state="failed", exit_code=status, error=str(error), results=command_results)
        write_json(record_path, record)
        print(f"Project worker failed: {error}", file=sys.stderr, flush=True)
        raise
    finally:
        # Terminating this job also ends the worker with the recorded status.
        # This reaps descendants left behind by an application on normal exit,
        # and cannot target another project or an unregistered process.
        api.TerminateJobObject(job, status)
        api.CloseHandle(job)


def launch(root, manifest, kind, commands, background=False, environment=None):
    for directory in (".local/logs", ".local/tmp", ".local/state"):
        inside(root, directory).mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    relative = f".local/state/plans/{token}.json"
    env = environment if environment is not None else project_environment(root, manifest)
    overrides = {key: value for key, value in env.items() if os.environ.get(key) != value}
    slot = "native" if kind == "build" else ("setup" if kind == "setup" else None)
    if kind == "run" and any(command[2:4] == ["-m", "hangar"] for command in commands):
        slot = "native"
    if kind == "run" and any(Path(command[0]).name == "flightsim-viewer.exe" for command in commands):
        slot = "viewer"
    write_json(inside(root, relative), {"schema": 1, "token": token, "kind": kind, "commands": commands, "environment": overrides, "lock": slot})
    command = [manifest["tools"]["python"]["path"], "-B", str(inside(root, "tools/bootstrap/project.py")), "--root", str(root), "_worker", relative]
    if background:
        # Python's CREATE_NO_WINDOW is the equivalent of Hidden for this direct
        # native process API; no Start-Process window can steal focus.
        log = inside(root, ".local/logs/" + token + ".log")
        with log.open("w", encoding="utf-8") as stream:
            process = subprocess.Popen(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
        registered = inside(root, ".local/state/processes/" + token + ".json")
        deadline = time.monotonic() + 10
        while not registered.is_file() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        if not registered.is_file():
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
            raise RuntimeError(f"Background worker did not register ownership; inspect {log}")
        _BACKGROUND_CHILDREN.append(process)
        print(json.dumps({"background_pid": process.pid, "token": token, "log": str(log)}), flush=True)
        return 0
    status = subprocess.call(command, cwd=root, env=env)
    api = windows_api()
    job = api.OpenJobObjectW(0x0004, False, job_name(root, token))  # JOB_OBJECT_QUERY
    if job:
        try:
            _wait_job_empty(api, job)
        finally:
            api.CloseHandle(job)
    elif ctypes.get_last_error() != 2:  # ERROR_FILE_NOT_FOUND: the job already ceased to exist.
        raise OSError(ctypes.get_last_error(), "Cannot verify completed project job termination.")
    return status


def stop(root):
    directory = inside(root, ".local/state/processes")
    if not directory.is_dir():
        print("No registered project processes.")
        return 0
    api = windows_api()
    stopped, ignored = [], []
    for path in sorted(directory.glob("*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        if record.get("state") != "running":
            continue
        live = process_identity(record.get("pid", 0))
        if not identity_matches(record, live, root) or record.get("job") != job_name(root, record.get("token", "")):
            ignored.append(record.get("pid"))
            continue
        job = api.OpenJobObjectW(0x000C, False, record["job"])  # JOB_OBJECT_TERMINATE | QUERY
        if not job:
            ignored.append(record["pid"])
            continue
        try:
            process = api.OpenProcess(0x1000, False, record["pid"])
            belongs = wintypes.BOOL(False)
            try:
                if not process or not api.IsProcessInJob(process, job, ctypes.byref(belongs)) or not belongs.value:
                    ignored.append(record["pid"])
                    continue
            finally:
                if process:
                    api.CloseHandle(process)
            if not api.TerminateJobObject(job, 130):
                raise OSError(ctypes.get_last_error(), "Owned project job could not be terminated.")
            _wait_job_empty(api, job)
            record.update(state="stopped", exit_code=130)
            write_json(path, record)
            stopped.append(record["pid"])
        finally:
            api.CloseHandle(job)
    print(json.dumps({"stopped_project_jobs": stopped, "stale_or_unmatched_records_ignored": ignored}))
    for child in list(_BACKGROUND_CHILDREN):
        if child.pid in stopped:
            child.wait(timeout=5)
            _BACKGROUND_CHILDREN.remove(child)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True)
    commands = parser.add_subparsers(dest="command", required=True)
    for setup_name in ("setup", "_setup"):
        setup_parser = commands.add_parser(setup_name)
        setup_parser.add_argument("--ucrt64-root")
        setup_parser.add_argument("--offline", action="store_true")
    build_parser = commands.add_parser("build")
    build_parser.add_argument("--model", action="append")
    build_parser.add_argument("--jobs", type=int, default=8)
    build_parser.add_argument("--memory-limit-mib", type=int, default=4096)
    build_parser.add_argument("--test", action="store_true")
    build_parser.add_argument("--plan", action="store_true")
    run_parser = commands.add_parser("run")
    run_parser.add_argument("--mode", choices=("viewer", "demo", "headless", "hangar", "python"), default="viewer")
    run_parser.add_argument("--preset", choices=("ucrt64-release", "ucrt64-headless"), default="ucrt64-release")
    run_parser.add_argument("--background", action="store_true")
    run_parser.add_argument("arguments", nargs=argparse.REMAINDER)
    commands.add_parser("stop")
    commands.add_parser("_stage-assets")
    worker_parser = commands.add_parser("_worker")
    worker_parser.add_argument("plan")
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    inside(root, ".local/state")
    if args.command == "_setup":
        return setup(root, args)
    if args.command == "setup":
        command = [sys.executable, "-B", str(inside(root, "tools/bootstrap/project.py")), "--root", str(root), "_setup"]
        for option in ("ucrt64_root",):
            if getattr(args, option):
                command += ["--" + option.replace("_", "-"), getattr(args, option)]
        if args.offline:
            command += ["--offline"]
        bootstrap_manifest = {"ucrt64_root": str(inside(root, ".local/msys2/ucrt64")), "tools": {"python": {"path": sys.executable}}}
        return launch(root, bootstrap_manifest, "setup", [command])
    if args.command == "_worker":
        return worker(root, args.plan)
    if args.command == "stop":
        return stop(root)
    manifest = load_manifest(root)
    if args.command == "_stage-assets":
        for item in load_lock(root).get("runtime_assets", []):
            source = fetch(root, item, offline=True)
            for relative in item["destinations"]:
                destination = inside(root, relative)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, destination)
        return 0
    if args.command == "build":
        if args.jobs < 1 or args.jobs > 128:
            raise ValueError("Build jobs must be between 1 and 128.")
        if args.memory_limit_mib < 1 or args.memory_limit_mib > 2147483647:
            raise ValueError("Hangar memory limit must be positive and fit a signed 32-bit MiB count.")
        plan = build_commands(root, manifest, args)
        if args.plan:
            print(json.dumps({"commands": plan, "pending": manifest.get("pending", [])}, indent=2))
            return 0
        return launch(root, manifest, "build", plan)
    if args.arguments and args.arguments[0] == "--":
        args.arguments = args.arguments[1:]
    env = project_environment(root, manifest)
    env["FSIM_PRESET"] = args.preset
    env["FSIM_DLL_DIR"] = str(inside(root, f"build/{args.preset}/bin"))
    if args.mode in ("python", "hangar"):
        env["PYTHONPATH"] = os.pathsep.join([str(inside(root, ".local/packages")), str(inside(root, "tools/models/hangar")), str(inside(root, f"build/{args.preset}/python"))])
    if args.mode == "hangar":
        env["HANGAR_MESHKIT_DIR"] = str(inside(root, f"build/{args.preset}/python"))
    return launch(root, manifest, "run", run_commands(root, manifest, args), args.background, env)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"Project bootstrap failed: {error}", file=sys.stderr, flush=True)
        raise SystemExit(1)
