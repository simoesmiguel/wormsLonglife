#!/usr/bin/env python
import subprocess

for (( c=1; c<=5; c++ ))
do 
	with open("output.txt", "w+") as output:
    	subprocess.call(["python", "./script.py"], stdout=output);
done 

