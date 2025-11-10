#!/usr/bin/env python3
"""
fix_ruff_issues.py
-------------------------------------------------
Batch-fix Ruff style issues and minor syntax problems
for the DriveSentinel project (v1 + v2), Windows-friendly.
"""

from pathlib import Path
import subprocess
import sys
import re

# Directories to scan
SEARCH_DIRS = ["pc", "dashboard", "v2"]


def ensure_tool(modname: str, pkgname: str) -> None:
    """Ensure a module is importable; if not, pip install it in current env."""
    try:
        __import__(modname)
    except Exception:
        print(f"🔧 Installing {pkgname} ...")
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "-U", pkgname], check=False
        )


def fix_file(path: Path):
    text = path.read_text(encoding="utf-8", errors="ignore")

    # Critical: broken multiline SQL string in v2/pc_v2/telemetry_store.py
    if "CREATE TABLE IF NOT EXISTS events" in text and 'DDL = "' in text:
        text = re.sub(
            r'DDL\s*=\s*".*?CREATE TABLE IF NOT EXISTS events.*?;"',
            'DDL = """\nCREATE TABLE IF NOT EXISTS events (\n  ts REAL,\n  type TEXT,\n  data TEXT\n);\n"""',
            text,
            flags=re.S,
        )

    # Bare except -> except Exception:
    text = re.sub(r"(?m)^\s*except\s*:\s*$", "except Exception:", text)

    # Remove semicolons at end of line
    text = re.sub(r";\s*(\r?\n)", r"\1", text)

    # Split ; between statements (rare in your repo, but safe)
    text = re.sub(r"(\S)\s*;\s*(\S)", r"\1\n\2", text)

    # Trim trailing whitespace
    text = re.sub(r"[ \t]+(\r?\n)", r"\1", text)

    path.write_text(text, encoding="utf-8")


def main():
    # Make sure we're in the project root (script next to pc/, v2/, etc.)
    here = Path.cwd()
    print(f"Working in: {here}")

    # Apply regex fixes
    for folder in SEARCH_DIRS:
        p = Path(folder)
        if not p.exists():
            continue
        for py in p.rglob("*.py"):
            fix_file(py)

    # Ensure tools, then run them via -m to avoid PATH issues
    ensure_tool("ruff", "ruff")
    ensure_tool("black", "black")

    print("🧹 Running Ruff and Black auto-fixers...")
    subprocess.run([sys.executable, "-m", "ruff", "check", ".", "--fix"], check=False)
    subprocess.run([sys.executable, "-m", "black", "."], check=False)

    print("✅ All done! Re-run Ruff to confirm a clean state:")
    print("   ", sys.executable, "-m ruff check .")


if __name__ == "__main__":
    main()
