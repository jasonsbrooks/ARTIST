#!/bin/bash -x

# A bash script to dump all [0,$1] artist postgres databases to files /tmp/artist_$i.tar

for i in `seq 0 $1`;
do
	pg_dump artist_$i -f /tmp/artist_$i.tar -F t
done 
