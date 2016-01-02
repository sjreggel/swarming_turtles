import csv
import re
import sys
import os.path

file_name = sys.argv[1]
ifile  = open(file_name, "rb")
reader = csv.reader(ifile)
basename = os.path.splitext(sys.argv[1])[0]
outfile = basename+"-out.txt"
ofile = open(outfile,'w')
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
foraging_robot = [False] * Elements
foraging_count = [0] * Elements
foraging_start = [0] * Elements
foraging_time = [0] * Elements
foraging_total = 0
starttime = 0
endtime = 0
runtime = 0
convergence_time = 0
convergence_time_first = 0
convergence_start = 0
robots_in_experiment = 0
convergence = False

def decode(strs):
    global Shepontime, Sheptotaltime
    global Shepherd_enabled, Shepherd_count
    global Barkcounter, BarkedAtRobot, AskedRobot, Askcounter
    global starttime, endtime, convergence_time, convergence_start, convergence, convergence_time_first
    global robots_in_experiment
    global foraging_robot, foraging_count, foraging_start, foraging_time

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
		print convergence_time_first
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
   
runtime = endtime - starttime

print "\n******************************************************************************"
print "Decoded " + file_name + " --> " + outfile
print "MaxRobots = ", MaxRobots
print "General:	Robots in experiment 		= ", robots_in_experiment
print "General: 	Runtime experiment (sec) 	= ", runtime
print "******************************************************************************"
print "Sherpherding:	times enabled 			= ", Shepherd_count
print "Sherpherding:	Total time (sec) 		= ",  Sheptotaltime
print "Barked: 	Times at Robots [1..MaxRobots] 	= ", BarkedAtRobot[1:]
print "Barked: 	Total amount 			= ", Barkcounter
print "Asked Food: 	Times per Robot [1..MaxRobots] 	= ", AskedRobot[1:]
print "Asked Food: 	Total times asked 		= ", Askcounter
print "Foraging Runs: 	per robots [1..MaxRobots] 	= ", foraging_count[1:]
print "Foraging Runs: 	Total 				= ", foraging_total
print "Foraging Time: 	per robot [1..MaxRobots] (sec) 	= ", foraging_time[1:]
print "Convergence: 	Total time (sec) 		= ",  convergence_time
print "Convergence: 	Time to first (sec) 		= ",  convergence_time_first
print "******************************************************************************"
print "calc: 	food deliveries per second 		= ",  runtime / foraging_total


print "******************************************************************************"

ifile.close()
ofile.close()

