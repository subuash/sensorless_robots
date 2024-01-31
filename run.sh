for i in {1000..10000..1000}
do
    for _ in {1..10}
    do
        python overlay.py $i
    done
done