# for i in {10..100..10}
# do
#     for _ in {1..10}
#     do
#         python duplicate.py $i 0 warehouse1 0 0
#     done
# done
# for i in {1000..10000..2000}
# do
#     for _ in {1..2}
#     do
#         python overlay.py $i 1 warehouse$1 0 0
#         python overlay.py $i 1 warehouse1_$1 0 10
#         python overlay.py $i 1 warehouse2_$1 0 15
#     done
# done
for i in 10 100 1000
do
    for _ in {1..2}
    do
        python overlay.py $i 1 100_Scorridor$1 0 0
        python overlay50.py $i 1 50_Scorridor50_$1 0 0
        python overlay25.py $i 1 25_Scorridor25_$1 0 0
    done
done



# Particles, Run Number, Output File Name, Variance Angle(1 - 100), Variance Path(1-100)