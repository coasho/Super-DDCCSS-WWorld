# Closed publication of whole-file model products

The normal Hangar `model3d.finish` writer stages each complete output in its
destination directory, flushes and synchronizes its data, closes the writer,
and then publishes the staged file. The GLB header, JSON padding, binary chunk,
geometry, liquid-boundary extras and UTF-8/LF manifest serialization are
unchanged. The `open_write` and `atomic_write_text` bodies remain unchanged.
Legacy `replace(source, destination)` now reuses the guarded publisher: its
signature, one patience interval, return value and ordinary error/refusal
behavior are retained. When a native fallback is unsupported or denied, this
legacy route retries/rethrows the original `os.replace` refusal, with the native
failure available as its cause. Only a legitimate mapped-file replacement adds
a successful path.

The same closed stream is used at 16 explicitly reviewed complete writer
contexts: `pipeline.write_text`, `Design.save`, the six engine/nozzle/propeller
contexts and main XML in `Design.build_with`, `damage.write`, `pieces.write`,
`rotorcraft.heli._write`, `Rotorcraft.save/write_hover`, the store batch JSON
record, and `model3d.refresh_engine_lines`. Table-cache, liquid sidecar/damage
annotation and existing atomic JSON/XML publishers benefit through common
`replace`; their serializers are not duplicated. The unchanged-file shortcuts,
date handling, namespaces, existing retirement rules and return values remain
authoritative. Text contexts retain their original encoding and newline
arguments, including contexts whose default newline translates on Windows.
General `open_write` semantics and other callers are not silently changed.

Ordinary publication uses `os.replace`. On Windows sharing or mapped-file
refusal, the new private publisher first requests DELETE access to the existing
destination with read/write/delete sharing. A reader that denies delete
sharing prevents this request. It then renames the closed staged file using
`SetFileInformationByHandle(FileRenameInfoEx)` with only
`FILE_RENAME_REPLACE_IF_EXISTS | FILE_RENAME_POSIX_SEMANTICS`. Existing mapped
readers retain their old data while a new open sees the complete new file.
Readonly-ignore, ACL-ignore, permission/attribute changes, unmapping other
readers, and process termination are not used. Named reparse entries are opened
with `FILE_FLAG_OPEN_REPARSE_POINT`.

One existing 20-second patience interval covers the ordinary attempt, native
fallback and sharing retries. Unsupported native operations fail explicitly;
there is no fallback to truncation. Owned handles close on all paths and the
owned staging pathname is removed on failure. The DELETE gate shares delete
access; it does not exclude a competing publisher. The project's single-writer
implementation/production window remains the serialization contract.

Publication is atomic **per file**. Separate GLB and manifest replacements are
not a two-file transaction or a power-failure durability guarantee. A later
manifest failure can leave a complete new GLB alongside the old manifest;
`finish` raises and the model stage is unsuccessful. No partial model receipt
may be admitted. The fixture explicitly covers this boundary.

The actual Windows fixture reproduces direct truncation failure on a permanently
mapped, delete-sharing target. Plain staged `os.replace` also failed with
WinError 5 on this host; the guarded native rename succeeds and the original
mapped view remains byte-identical. A nondelete-sharing mapping refuses
publication and preserves old destination bytes. Other controls cover actual
`finish` bytes, closed-before-rename ordering, write/flush/fsync/replace failures,
staging cleanup, unsupported operations and UTF-16 paths including a surrogate
pair. A separate real mapped-text fixture reproduced the same failure through
`atomic_write_text` before its common publisher was extended. All 16 actual
writer context bodies are executed against ordinary-file byte oracles and
mapped-file publication. Additional full-function fixtures verify damage
precision/rounding, piece paths, table-cache bytes, both save methods' metadata
and default newlines, hover text, manifest refresh, unchanged-file shortcuts,
serializer failure cleanup and original legacy refusal identity/deadline.
These are self-contained tests loaded by the mandatory `model_methods.py`
suite, without a diagnostic driver or aircraft generation. They do not replace
actual model generation or packing acceptance. The failed RQ-4B normal writer
and its earlier independent packing refusal remain separate evidence; actual
normal production must be rerun.

Primary API references:

- [CreateFileW access and sharing rules](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilew).
- [SetFileInformationByHandle](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-setfileinformationbyhandle) and [FileRenameInfoEx class](https://learn.microsoft.com/en-us/windows/win32/api/minwinbase/ne-minwinbase-file_info_by_handle_class).
- [Rename flags and existing-handle behavior](https://learn.microsoft.com/en-us/windows-hardware/drivers/ddi/ntifs/ns-ntifs-_file_rename_information).
- [ReplaceFileW failure states](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-replacefilew), which can include partial multi-step replacement and are not used by this helper.

These APIs depend on Windows/filesystem support. Unsupported systems retain
their explicit refusal; no compatibility or equipment-accuracy claim follows
from the local Windows fixture.
