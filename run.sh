for i in {1000..10000..1000}
do
    for _ in {1..2}
    do
        python overlay.py $i 1 warehouse$1 10 0
        python overlay.py $i 1 warehouse1_$1 50 0
        python overlay.py $i 1 warehouse2_$1 100 0
    done
done




# Particles, Run Number, Output File Name, Variance Angle(1 - 100), Variance Path(1-100)