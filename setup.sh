#!/bin/bash

## HW4 Setup Script.
## Places the HW4 code into the ~/homework_ws and builds the packages.

## Preconditions:
## - The homework_4 repo was downloaded as a Zip file from GitHub to ~/Downloads
## - The ~/homework_ws/src folder has been created, as per the pre-HW instructions.

cd ~/Downloads
unzip hw4_planning-main.zip
mv ~/Downloads/hw4_planning-main ~/homework_ws/src
mv ~/homework_ws/src/hw4_planning-main ~/homework_ws/src/hw4_planning
cd ~/homework_ws
catkin build
source ~/homework_ws/devel/setup.bash
