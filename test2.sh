number=(5 10 15 20 25 30 40 50)
algorithms=("ORCA" "BAPF" "EAPF")

for j in "${algorithms[@]}"
do
    for k in "${number[@]}"
    do
        python generate.py -n $k -a $j -t 3
        gzserver drones_$k.world >> /home/matteo/test_paper/collisions/collisions_${k}_${j}_3.txt &
        sleep `echo "($k*2)*60" | bc -l`
        kill -15 `pidof gzserver`
        echo "Ucciso il processo test test: 3, algo: $j, droni: $k"
    done
    sleep 10s
done