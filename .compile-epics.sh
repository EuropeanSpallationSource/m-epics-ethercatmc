#!/usr/bin/env bash

# shellcheck disable=SC2268

# bash script to be able to run travis locally
# Make it possible to test a travis run locally,
# before pushing the branch

#Setting environment variables, may be overwritten by
# e.g. .travis.yml

if [ "x$SETUP_PATH" = x ]
then
  export SETUP_PATH=.ci-local:.ci
fi
# shellcheck disable=SC2068
if [ "x$SET" = x ]
then
  export SET=latest
fi
# shellcheck disable=SC2068
if [ "x$BASE" = x ]
then
  export BASE=7.0
fi

# shellcheck disable=SC2068
if [ "x$CACHEDIR" = x ]
then
  export CACHEDIR=$HOME/.cache/travis/EPICS
fi

if [ "x$MOTOR_RECURSIVE" = x ]
then
  export MOTOR_RECURSIVE=NO
fi

env

python .ci/cue.py prepare
