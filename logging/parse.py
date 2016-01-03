import csv
import re
import sys
import os.path
from operator import sub
import math

file_name = sys.argv[1]
ifile  = open(file_name, "rb")
reader = csv.reader(ifile)
basename = os.path.splitext(sys.argv[1])[0]
outfile = basename+"-out.txt"
ofile = open(outfile,'w')
logfile = basename+"-out.log"
lfile = open(logfile,'w')

Shepontime = 0
Sheptotaltime = 0
Shepherd_enabled = False
Shepherd_count = 0
Barkcounter = 0
MaxRobots = 10
Elements = MaxRobots+1
BarkedAtRobot = [0] * Elements
AskedRobot = [0] * Elements
Askcounter = 0
Receivedfood = [0] * Elements
Sendfood = [0] * Elements
Receivecounter = 0
foraging_robot = [False] * Elements
foraging_count = [0] * Elements
foraging_start = [0] * Elements
foraging_time = [0] * Elements
foraging_total = 0
hasfood_robot = [False] * Elements
hasfood_count = [0] * Elements
hasfood_start = [0] * Elements
hasfood_time = [0] * Elements
hasfood_total = 0
starttime = 0
endtime = 0
runtime = 0
convergence_time = 0
convergence_time_first = 0
convergence_start = 0
robots_in_experiment = 0
convergence = False
prev_xpos = [0] * Elements
prev_ypos = [0] * Elements
distance = [0] * Elements

def decode(strs):
    global Shepontime, Sheptotaltime
    global Shepherd_enabled, Shepherd_count
    global Barkcounter, BarkedAtRobot, AskedRobot, Askcounter, Receivedfood, Receivecounter, Sendfood
    global starttime, endtime, convergence_time, convergence_start, convergence, convergence_time_first
    global robots_in_experiment
    global foraging_robot, foraging_count, foraging_start, foraging_time
    global hasfood_robot, hasfood_count, hasfood_start, hasfood_time
    global prev_xpos, prev_ypos, distance

    #decode the raw data
    res = [","] * 10
    res = re.findall(r"\"(.*?)\"", strs, re.DOTALL)[2:3]
    res += re.findall(r"'(.*?)'", strs, re.DOTALL)
    res += re.findall(r"> (.*?);", strs, re.DOTALL)
    mylist = strs.split(" ->", 1)[0]
    mylist = mylist.replace(')',',')
    mylist = mylist.replace(" ","")
    res += mylist.split(',')[5:8]

    #write to output file
    for item in res:
      try:
        ofile.write("%s," % item)
      except:
	pass
    ofile.write("\n")

 
    #map results to variables
    try:
      time = res[0]
      robot = res[1]
      xpos = res[2]
      ypos = res[3]
      zpos = res[4]
      wpos = res[5]
      action = res[6]
    except:
	pass
    try:
	foraging = res[7]
    except:
	foraging = ""
    try:
	foodcount = res[8]
    except:
	foodcount = ""
    try:
	hasfood = res[9]
    except:
	hasfood = ""
    #print time, robot, xpos, ypos, zpos, wpos, action, foraging, foodcount, hasfood

    #log experiment time
    try:
     if action == 'Starting Experiment':
	print "statrting"
  	starttime = float(time)
     endtime = float(time)
    except:
	pass

    #count the amount of robots in experiment
    try: 
	if "_" in robot:
    	  param, value = robot.split("_",1)
	  current_robot = int(value)
	  if robots_in_experiment < current_robot:
	    robots_in_experiment =  current_robot
    except:
	pass


    # log shepherding time and count
    try:
      if action == 'Shepherding enabled! ':
	Shepherd_enabled =  True
	Shepontime = float(time)
	Shepherd_count += 1
    except:
	pass
    try:
      if action == 'Shepherding disabled ':
	Sheptotaltime += float(time) - Shepontime
	Shepherd_enabled =  false
    except:
	pass

    #log the received barks of each robot
    try: 
      if action.startswith('Barked at robot'):
	Barkcounter += 1
	if "_" in action:
    	  param, value = action.split("_",1)
	  BarkedAtRobot[int(value)] += 1
    except:
	pass

    #log asking food for each robot
    try: 
      if action.startswith('asking robot_'):
	Askcounter += 1
	AskedRobot[current_robot] += 1
    except:
	pass

    #log received / send food for each robot
    try: 
      if action.startswith('Foodlocation received from robot_'):
	Receivecounter += 1
	Receivedfood[current_robot] += 1
	if "_" in action:
    	  param, value = action.split("_",1)
	  Sendfood[int(value)] += 1
    except:
	pass



    # log foraging and convergence time and count per robot
    try:
      if foraging == 'True':
	if foraging_robot[current_robot] == False: #start new forage run(s)
	  foraging_count[current_robot] += 1
	  foraging_robot[current_robot] =  True
	  foraging_start[current_robot] = float(time)
	  for robotnr in range(1,robots_in_experiment): # check if all robots are currently foraging
	      if foraging_robot[robotnr] != True:
		convergence = False
		break
	      else:
		convergence = True
	  if convergence == True:
		convergence_start = float(time)
		if convergence_time_first == 0: # time till first convergence
		  convergence_time_first = float(time)
      else: 
	if foraging_robot[current_robot] == True: #end forage run
	  foraging_robot[current_robot] =  False
	  foraging_time[current_robot] += float(time) - foraging_start[current_robot]
	  if convergence == True: # end of convergence
		convergence_time += float(time) - convergence_start
	  convergence = False
    except:
	pass


    # log overall convergence
    try:
      if foraging == 'True':
	if foraging_robot[current_robot] == False: #start forage run
	  foraging_count[current_robot] += 1
	  foraging_robot[current_robot] =  True
	  foraging_start[current_robot] = float(time)
      else: 
	if foraging_robot[current_robot] == True: #end forage run
	  foraging_robot[current_robot] =  False
	  foraging_time[current_robot] += float(time) - foraging_start[current_robot]
    except:
	pass


    # log hasfood time and count per robot
    try:
      if hasfood == 'True':
	if hasfood_robot[current_robot] == False: #got new food
	  hasfood_count[current_robot] += 1
	  hasfood_robot[current_robot] =  True
	  hasfood_start[current_robot] = float(time)
      else: 
	if hasfood_robot[current_robot] == True: #delivered or dropped food
	  hasfood_robot[current_robot] =  False
	  hasfood_time[current_robot] += float(time) - hasfood_start[current_robot]
    except:
	pass


    # log travel distance per robot
    try:
	if robot == 'mitro':
	   current_robot = 0
	distance[current_robot] += math.sqrt(math.pow((float(xpos) - prev_xpos[current_robot]),2) + math.pow((float(ypos) - prev_ypos[current_robot]),2))
	prev_xpos[current_robot] = float(xpos)
	prev_ypos[current_robot] = float(ypos)
    except:
	pass



# write column lables to file
ofile.write("time, robot, xpos, ypos, zpos', wpos, action, foraging, foodcount, hasfood \n")


# iterate trough the file and decode
rownum = 0
for row in reversed(list(reader)):
    # Save header row.
    if rownum == 0:
        header = row
    else:
        colnum = 0
        for col in row:
            #print '%-8s: %s' % (header[colnum], col)
	    if colnum == 0:
	    	decode(col)
            colnum += 1
    rownum += 1


#adjust foraged time when still foraging at end of experiment
for robotnr in range(1,MaxRobots):
  if foraging_robot[robotnr] == True: #robot still foraging
     foraging_time[robotnr] += endtime - foraging_start[robotnr] 
     print "correcting foraging time"
#adjust convergence time when still converged at end of experiment
if convergence == True: 
   convergence_time += endtime - convergence_start
   print "correcting convergence time"

#total forage runs
for robotnr in range(1,MaxRobots):
  foraging_total += foraging_count[robotnr]
  hasfood_total += hasfood_count[robotnr]
   
runtime = endtime - starttime

lfile.write("\n****************************************************************************************\n")
lfile.write( "Decoding " + file_name + " --> %s \n" % outfile)
lfile.write( "General: 		MaxRobots 			= %s \n" % MaxRobots )
lfile.write( "General:		Robots in experiment 		= %s \n" % robots_in_experiment)
lfile.write( "General: 		Runtime experiment (sec) 	= %s \n" % runtime)
lfile.write( "****************************************************************************************\n")
lfile.write( "Sherpherding:		Times enabled 			= %s \n" % Shepherd_count)
lfile.write( "Sherpherding:		Total enabled time (sec) 	= %s \n" %  Sheptotaltime)
lfile.write( "Barked: 		Total amount 			= %s \n" % Barkcounter)
lfile.write( "Barked: 		Times at Robots [1..MaxRobots] 	= %s \n" % BarkedAtRobot[1:])
lfile.write( "Asked Foodloc: 		Total times asked 		= %s \n" % Askcounter)
lfile.write( "Asked Foodloc: 		Times per Robot [1..MaxRobots] 	= %s \n" % AskedRobot[1:])
lfile.write( "Received Foodloc: 	Total times Received 		= %s \n" % Receivecounter)
lfile.write( "Received Foodloc: 	Times per Robot [1..MaxRobots] 	= %s \n" % Receivedfood[1:])
lfile.write( "Send Foodloc: 		Times per Robot [1..MaxRobots] 	= %s \n" % Sendfood[1:])
lfile.write( "Foraging Runs: 		Total 				= %s \n" % foraging_total)
lfile.write( "Foraging Runs: 		per robots [1..MaxRobots] 	= %s \n" % foraging_count[1:])
lfile.write( "Foraging Time: 		per robot [1..MaxRobots] (sec) 	= %s \n" % foraging_time[1:])
lfile.write( "Got food: 		Total 				= %s \n" % hasfood_total)
lfile.write( "Got food: 		per robots [1..MaxRobots] 	= %s \n" % hasfood_count[1:])
lfile.write( "Got food Time: 		per robot [1..MaxRobots] (sec) 	= %s \n" % hasfood_time[1:])
lfile.write( "Dropped food: 		Total 				= %s \n" % (hasfood_total - foraging_total))
lfile.write( "Dropped food: 		per robots [1..MaxRobots] 	= %s \n" % map(sub,hasfood_count[1:], foraging_count[1:]))
lfile.write( "Convergence: 		Total time converged (sec)	= %s \n" % convergence_time)
lfile.write( "Convergence: 		Time to first (sec) 		= %s \n" % convergence_time_first)
lfile.write( "Distance travelled:	per robots [1..MaxRobots] (m)	= %s \n" % distance[1:])
lfile.write( "Distance travelled:	mitro (m)			= %s \n" % distance[0])
lfile.write( "****************************************************************************************\n")
lfile.write( "calc troughput: 	food deliveries per second 	= %s \n" %  (runtime / foraging_total))
lfile.write( "****************************************************************************************\n")




ifile.close()
ofile.close()
lfile.close()


with open(logfile, 'r') as fin:
    print fin.read()

