#!/bin/bash
set -e

# Find the package root directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PKG_ROOT="$(dirname "$DIR")"

echo "=== Formatting C++ files (clang-format) ==="
find "$PKG_ROOT/src" "$PKG_ROOT/include" -name "*.cpp" -o -name "*.hpp" | xargs -r clang-format -i --style=file

echo "=== Formatting Python files (black) ==="
if command -v black &> /dev/null; then
    black --config "$PKG_ROOT/pyproject.toml" "$PKG_ROOT/scripts"/*.py
else
    echo "black not found. Skipping python formatting. Run 'pip install black' or use nix-shell to format."
fi

echo "=== Done Formatting! ==="
