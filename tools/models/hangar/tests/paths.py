"""Project-contained temporary output shared by Hangar test processes."""
from pathlib import Path
import os


def temporary_root():
    root=Path(os.environ.get('FSIM_PROJECT_ROOT') or Path(__file__).absolute().parents[4])
    if any(part.casefold()=='.codex' for part in root.parts):raise ValueError('Excluded project temporary root')
    root=root.resolve()
    if any(part.casefold()=='.codex' for part in root.parts):raise ValueError('Excluded resolved project temporary root')
    result=(root/'.local/tmp').resolve()
    if any(part.casefold()=='.codex' for part in result.parts):raise ValueError('Excluded resolved project temporary output')
    if not result.is_relative_to(root):raise ValueError('Project temporary output escaped its root')
    result.mkdir(parents=True,exist_ok=True)
    return result


TEMP_ROOT = temporary_root()
