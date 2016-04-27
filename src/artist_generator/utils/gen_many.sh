#!/bin/bash

for i in `seq 1 $1`
do
	python generate.py -n ngram/model/ -g 800 > /dev/null &
	echo "Starting $i, job $!"
done

trap 'kill $(jobs -p)' EXIT

wait

