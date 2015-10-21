#!/bin/bash
node_process_id=$(pidof nodelet)
while [ -z "$node_process_id" ]
do
    sleep 1
    node_process_id=$(pidof nodelet)
done

cpulimit --lazy --limit $1 --exe nodelet 
