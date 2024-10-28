#!/bin/bash

export port=$1
export conf=$2

#echo $port
#echo $conf
#echo 'mosquitto -v -p "$port" -c "$conf"'
mosquitto -v -p "$port" -c "$conf"
