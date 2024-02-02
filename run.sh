for i in {1000..10000..1000}
do
    for _ in {1..2}
    do
        python overlay.py $i 1 warehouse$1
    done
done