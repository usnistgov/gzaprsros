#!/bin/bash
numbers=(`cat version`)


let numbers[2]++
echo ${numbers[2]}
 
echo ${numbers[0]} ${numbers[1]} ${numbers[2]} > version 
#echo "$number"  > version 
