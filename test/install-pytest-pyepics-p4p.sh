#!/bin/sh
set -e -x

sudo apt-get install pip3 || :
sudo apt-get install python3-pip || :
sudo pip3 install pytest || :
sudo pip3 install pyepics || :
sudo pip3 install p4p || :
