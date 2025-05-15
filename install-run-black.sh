#!/bin/sh

#Check black, the python formatter
BLACK_VERSION=23.9.1
if ! black --version | grep -q "[^0-9]${BLACK_VERSION}[^0-9]"; then
  pip install git+https://github.com/psf/black@$BLACK_VERSION
fi
if ! black --version | grep -q "[^0-9]${BLACK_VERSION}[^0-9]"; then
  echo >&2 black not found or wrong version
  exit 1
fi
# shellcheck disable=SC2035
TERM=dumb black --check test/pytests36/*.py

