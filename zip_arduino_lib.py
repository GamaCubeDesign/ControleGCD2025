#!/usr/bin/env python3

"""
Script for creating a .zip file from Arduino library
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
    "*.py"
    ".pyc",
    ".tmp",
    "*.md",
    "*.zip",
]

def should_exclude(path: Path) -> bool:
    """
    Exclude a path from EXCLUDE_PATTERNS

    Parameter:
    - path: Path, users path

    Returns:
    - rel_str: Bool, tells if a file is from any file format found in EXCLUDE_PATTERNS
    """
    rel_str = str(path)

    for pattern in EXCLUDE_PATTERNS:
        if fnmatch.fnmatch(rel_str, pattern) or fnmatch.fnmatch(path.name, pattern):
            return True
    return False

def main():
    lib_root = Path(__file__).resolve().parent
    home_dir = Path.home()
    zip_path = home_dir / ZIP_NAME

    print(f"Library dir: {lib_root}")
    print(f"Zip will be: {zip_path}")

    with ZipFile(zip_path, "w", compression=ZIP_DEFLATED) as zf:
        for path in lib_root.rglob("*"):
            # Jumps the scipt it self
            if path == Path(__file__).resolve():
                continue
            rel_path = path.relative_to(lib_root)
            if should_exclude(rel_path):
                continue
            if path.is_file():
                print(f"Adding: {rel_path}")
                zf.write(path, arcname=rel_path)
    print("Done")
    print(f"File created at: {zip_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Keyboard interrput by user\n")
        sys.exit(1)
