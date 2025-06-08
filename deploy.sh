#!/bin/bash

cd build/
make -j
scp src/sparkie_fw.uf2 sparkie@sparkie.local:~/
# Wait for user to press enter
read -p "Ensure that pico is in bootloader mode and press enter to continue..."
ssh sparkie@sparkie.local "picotool load -xu sparkie_fw.uf2"
cd ../