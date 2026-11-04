# Installed per-model component execution

`component_public.py` executes 24 distinct AAM models in each of C++, C99 and
Python: 72 actual installed SDK processes. C++ uses `World` and `Agent`; C99 uses
permission-scoped public handles; Python imports the installed facade and native
extension. Clients contain scenario setup, requests and observations. They do
not calculate seeker measurements, guidance, control, motor force, fuzes or
damage.

The fixed public fixture was recorded before its first execution: seed19,
one worker, native dt1/120s, frame skip4, 6000m altitude, 250m/s head-on aircraft,
5000m initial separation and 40s after release. A maintained legal carrier and
station are selected from the actual fit/payload records. An actor obtains its
own radar designation and requests a strike. An approving C2 Agent receives and
answers the modeled DMS consent request. Referee state, fin diagnostics and
events remain separate from those actor targeting inputs.

Each process must identify the actual released store and native model, observe
real tracking, burning and controller activity, retain every native decision
sample, and finish with an actual detonation and matching target damage events.
Misses fail this fixture and retain their raw data. No outcome is synthesized.
The existing native seed17 fixture, countermeasure fleet scenes, calibrated
data and visual scenes are independent gates and remain unchanged.

The native consumers hash their actual executable and loaded SDK/JSBSim DLLs
and both checked-closed observation files with Windows CNG SHA256. The Python
consumer also identifies and hashes its loaded extension. Prepared binary hashes
are reconciled to the original installer, rather than adopted from arbitrary
current prefix contents. Consumed aircraft resources are rechecked against the
same normal producer's product manifest. The input epoch covers actual component
sources, native resources, normal generation code, used facade import closure
and fit/payload selection inputs.

Read-only `admit()` independently reconstructs all 72 processes, exact commands,
runtime identities, source epoch, trace clocks/counts, closed-file hashes and
target events before producing 216 public/installed component records. A
diagnostic `--only-model` invocation cannot produce that proof. The final report
requires the exact current campaign's proof and row identity for every cell.
An installed runtime pass establishes interface/product execution; equipment
calibration retains its separate per-model ledger.

Repository refusal controls use explicitly synthetic files and never count as
installed simulation execution. The checksum helper has a separately executed
C99 `abc` known-vector check. [Microsoft's CNG contract](https://learn.microsoft.com/en-us/windows/win32/api/bcrypt/nf-bcrypt-bcryptcreatehash)
defines the hash-object lifetime and supported Windows allocation behavior.
