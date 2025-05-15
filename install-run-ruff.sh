#!/bin/sh
# Check ruff, the fast Python linter and code formatter
# Note: The version without 'v'. 'v' is used when installing
set -x
RUFF_VERSION=0.11.9
if ! ruff --version | grep -q "[^0-9]${RUFF_VERSION}$"; then
  pip install git+https://github.com/charliermarsh/ruff-pre-commit@v$RUFF_VERSION
fi
if ! ruff --version | grep -q "[^0-9]${RUFF_VERSION}$"; then
  echo >&2 ruff not found or wrong version
  exit 1
fi
# shellcheck disable=SC2035
NO_COLOR=1 ruff check test/pytests36/*.py
