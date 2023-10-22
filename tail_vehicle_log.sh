#!/bin/bash

cd ~/Results

# Find the newest "vehicle_log.txt*" file
newest_file=$(ls -t *vehicle_log.txt | head -n 1)

# If the file is found, tail -f that file
if [ ! -z "$newest_file" ]; then
    tail -f "$newest_file"
else
    echo "No '*vehicle_log.txt' files found in ~/Results."
fi
