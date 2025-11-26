#!/usr/bin/env python3

"""
Script for creating a .zip file from Arduino library
Ensures the .zip has a single top-level directory (required by Arduino IDE).
"""

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile
import fnmatch
import sys

# Config
ZIP_NAME = "IaraSat.zip"

EXCLUDE_PATTERNS = [
    ".git*",
    ".vscode",
    "__pycache__",
    "*.py",
    "*.pyc",
    "*.tmp",
    "*.md",
    "*.zip",
]

def should_exclude(path: Path) -> bool:
    """
    Exclude a path from EXCLUDE_PATTERNS

    Parameter:
    - path: Path, users path

    Returns:
    - Bool, tells if a file matches EXCLUDE_PATTERNS
    """
    rel_str = str(path)

    for pattern in EXCLUDE_PATTERNS:
        if fnmatch.fnmatch(rel_str, pattern) or fnmatch.fnmatch(path.name, pattern):
            return True
    return False


def main():
    lib_root = Path(__file__).resolve().parent
    lib_root_name = lib_root.name  # <-- Pasta superior dentro do ZIP

    home_dir = Path.home()
    zip_path = home_dir / ZIP_NAME

    print(f"Library dir: {lib_root}")
    print(f"Top folder in ZIP: {lib_root_name}/")
    print(f"Zip will be created at: {zip_path}")

    with ZipFile(zip_path, "w", compression=ZIP_DEFLATED) as zf:
        for path in lib_root.rglob("*"):

            # Ignore this script itself
            if path == Path(__file__).resolve():
                continue

            rel_path = path.relative_to(lib_root)

            if should_exclude(rel_path):
                continue

            if path.is_file():

                # >>> ALTERAÇÃO IMPORTANTE <<<
                # Tudo dentro da pasta lib_root_name/
                arcname = Path(lib_root_name) / rel_path

                print(f"Adding: {arcname}")
                zf.write(path, arcname=arcname)

    print("\nDone")
    print(f"File created at: {zip_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrupt by user\n")
        sys.exit(1)
