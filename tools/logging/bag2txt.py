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
import signal
from subprocess import Popen, PIPE


os.nice(20)

def terminate_process_and_children(p):
    try:
        ps_command = Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=PIPE)
        #print ("ps -o pid --ppid %d --noheaders" % p.pid)
        ps_output = ps_command.stdout.read()
        retcode = ps_command.wait()
        assert retcode == 0, "ps command returned %d" % retcode
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()
    except Exception as e:
        print('something went wrong during shutdown %s', e)

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
        fileName = string.rstrip(bagName, ".bag")


        

        msg = str("rm "+ fileName +".txt")
        os.system(msg)
        #echo_process = Popen(["rostopic", "echo", "/logging", ">|", logfile])
        cmd = str("rostopic echo /logging >| " + fileName + ".txt")
        echo_process = Popen(['/bin/bash', '-c', cmd])
        
        #msg = str("rostopic echo /logging >| "+ fileName +".txt &")
        #print msg
        #os.system(msg)
        time.sleep(1)
        msg = str("rosbag play -r 100 " + bagName)
        os.system(msg)
        time.sleep(1)
        #msg = str("kill %-")
        #os.system(msg)
        terminate_process_and_children(echo_process)
        time.sleep(1)

        msg = str("python ../scripts/parse.py "+ fileName + ".txt")
        os.system(msg)
        
        folder = "output"
        try: 
                os.makedirs(folder)
        except:
                pass

        shutil.move(fileName+"-out.log", folder + '/' + fileName+"-out.log")
        
        msg = str("rm "+ fileName +".txt")
        os.system(msg)
        msg = str("rm "+ fileName +"-out.txt")
        os.system(msg)

        
print "Done reading all " + numberOfFiles + " bag files."
