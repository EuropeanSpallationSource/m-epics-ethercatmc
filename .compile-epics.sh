#!/usr/bin/env bash

# bash script to be able to run travis locally
# Make it possible to test a travis run locally,
# before pushing the branch

#Setting environment variables, may be overwritten by
# e.g. .travis.yml
if [ x$SETUP_PATH = x ]
then
  export SETUP_PATH=.ci-local:.ci
fi
if [ x$SET = x ]
then
  export SET=latest
fi
if [ x$BASE = x ]
then
  export BASE=7.0
fi

if [ x$CACHEDIR = x ]
then
  export CACHEDIR=$HOME/.cache/travis/EPICS
fi

if [ x$MOTOR_RECURSIVE = x ]
then
  export MOTOR_RECURSIVE=NO
fi

env

python .ci/cue.py prepare
