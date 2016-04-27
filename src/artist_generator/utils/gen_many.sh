#!/bin/bash -x

for i in `seq 1 $1`
do
	python generate.py -n ngram/model/ -g 400 > /dev/null &
	echo "Starting $i, job $!"
done

trap 'kill $(jobs -p)' EXIT

wait

