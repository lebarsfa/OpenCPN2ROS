#!/bin/bash


echo "Openning ports ttyVUSB0 and ttyVUSB1"
socat -d -d -d -d PTY,raw,echo=0,link=/dev/ttyVUSB0  PTY,raw,echo=0,link=/dev/ttyVUSB1 >& /dev/null &
socat -d -d -d -d PTY,raw,echo=0,link=/dev/ttyVUSB2  PTY,raw,echo=0,link=/dev/ttyVUSB3 >& /dev/null &

sleep 5
sudo chmod 777 /dev/ttyVUSB0
sudo chmod 777 /dev/ttyVUSB1
sudo chmod 777 /dev/ttyVUSB2
sudo chmod 777 /dev/ttyVUSB3

echo "Port ready"
