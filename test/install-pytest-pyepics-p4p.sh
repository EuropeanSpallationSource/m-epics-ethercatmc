#!/bin/sh
set -e -x

sudo pip install pytest
sudo pip install pyepics
sudo pip install p4p

pip install pytest
pip install pyepics
pip install p4p

sudo apt-get install pip3 || :
sudo pip3 install pytest || :
sudo pip3 install pyepics || :
sudo pip3 install p4p || :
