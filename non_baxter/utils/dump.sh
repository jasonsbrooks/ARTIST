#!/bin/bash -x

for i in `seq 0 $1`;
do
	pg_dump artist_$i -f /tmp/artist_$i.tar -F t
done 
