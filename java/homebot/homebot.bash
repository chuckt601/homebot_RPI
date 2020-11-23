#!/bin/bash
#Exec=gnome-terminal -e
$echo "im working on it"
#"cd ~/homebot/java/homebot"
#java -cp .  ReverseServerThreaded 6868
#gnome-terminal -e java -cp jSerialComm-2.6.2.jar:. homebot
gnome-terminal -- java -cp ~/homebot/java/homebot/jSerialComm-2.6.2.jar:. homebot
