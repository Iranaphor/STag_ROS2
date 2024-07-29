#!/bin/bash

export port=$MQTT_BROKER_PORT

if [ "$port" -ne 1883 ]; then
    mosquitto -v -p "$port"
fi
