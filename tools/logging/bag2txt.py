'''
This script saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided

Usage1 (for one bag file):
	python bag2csv.py filename.bag
Usage 2 (for all bag files in current directory):
	python bag2csv.py

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon 

'''

import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file

#verify correct input arguments: 1 or 2
if (len(sys.argv) > 2):
	print "invalid number of arguments:   " + str(len(sys.argv))
	print "should be 2: 'bag2csv.py' and 'bagName'"
	print "or just 1  : 'bag2csv.py'"
	sys.exit(1)
elif (len(sys.argv) == 2):
	print sys.argv[1]
	listOfBagFiles = [sys.argv[1]]
        numberOfFiles = str(1)
	print "reading only 1 bagfile: " + str(listOfBagFiles[0])
elif (len(sys.argv) == 1):
	listOfBagFiles = [f for f in os.listdir(".") if f[-4:] == ".bag"]	#get list of only bag files in current dir.
	numberOfFiles = str(len(listOfBagFiles))
	print "reading all " + numberOfFiles + " bagfiles in current directory: \n"
	for f in listOfBagFiles:
		print f
	print "\n press ctrl+c in the next 10 seconds to cancel \n"
	time.sleep(1)
else:
	print "bad argument(s): " + str(sys.argv)	#shouldnt really come up
	sys.exit(1)

count = 0
for bagFile in listOfBagFiles:
	count += 1
	print "reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile
	#access bag
	bag = rosbag.Bag(bagFile)
	bagContents = bag.read_messages()
	bagName = bag.filename

        msg = str("rm "+ bagName +".txt")
        os.system(msg)
        msg = str("rostopic echo /logging >| "+ bagName +".txt &")
        print msg
        os.system(msg)
        msg = str("rosbag play -r 100 " + bagName)
        os.system(msg)
        msg = str("kill %-")
        msg = str("python parse.py "+ bagName + ".txt")
        os.system(msg)
        
        

print "Done reading all " + numberOfFiles + " bag files."
