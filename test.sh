#!/bin/bash
number=(5 10 15 20 25 30 40 50)
algorithms=("ORCA" "BAPF" "EAPF")
tests=(1 2 3)
for i in "${tests[@]}"
do
    for j in "${algorithms[@]}"
    do
        for k in "${number[@]}"
        do
            python generate.py -n $i -a $j -t $k
            gzserver drones_$i.world &
            sleep 190
            kill -9 `pidof gzserver`
            echo "Ucciso il processo $i $j $k"
        done
        sleep 10s
    done
done