"""Make a runnable local source snapshot beside experiment recordings."""
from pathlib import Path
import shutil

from fr3imp.model import ROOT


def capture_source(output):
    source=Path(output)/'source';source.mkdir(parents=True,exist_ok=True)
    shutil.copytree(ROOT/'fr3imp',source/'fr3imp',dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__'))
    vendor=source/'franka_fr3';vendor.mkdir(exist_ok=True)
    for name in ('fr3.xml','LICENSE'):shutil.copy2(ROOT/'franka_fr3'/name,vendor/name)
    if not (vendor/'assets').exists():
        (vendor/'assets').symlink_to(ROOT/'franka_fr3/assets',target_is_directory=True)
    (source/'README.txt').write_text('Run the project Python with this directory as cwd to use the recorded source.\n'
        'Vendor XML is copied; mesh assets are linked to the unchanged local vendor directory.\n')
