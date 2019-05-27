#!/bin/bash

rostopic pub -r 10 /chair_control intelchair/ChairMsg "battery: 0 
velocity: $1 
connected: 0"