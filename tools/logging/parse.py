import csv
import re
import sys
import os.path
from operator import sub
import math

file_name = sys.argv[1]
ifile  = open(file_name, "rb")
#reader = csv.reader(ifile)
reader = open(file_name, 'r')
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
Receivecounter = 0
Sendfood = [0] * Elements

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

collision_robot = [False] * Elements
collision_count = [0] * Elements
collision_start = [0] * Elements
collision_time = [0] * Elements
collision_total = 0


starttime = 0
endtime = 0
runtime = 0
convergence_time = 0
convergence_time_first = 0
convergence_start = 0
convergence_counter = 0
robots_in_experiment = 0
convergence = False
prev_xpos = [0] * Elements
prev_ypos = [0] * Elements
distance = [None] * Elements

food_xpos = 0
food_ypos = 0
food_zpos = 0
Food_moved = False
convergence_time_food = [0] * Elements
conv_start_time_food = 0
New_Foodfound = False

def decode(strs):
    global Shepontime, Sheptotaltime
    global Shepherd_enabled, Shepherd_count
    global Barkcounter, BarkedAtRobot, AskedRobot, Askcounter, Receivedfood, Receivecounter, Sendfood
    global starttime, endtime, convergence_time, convergence_start, convergence, convergence_time_first, convergence_counter
    global robots_in_experiment
    global foraging_robot, foraging_count, foraging_start, foraging_time
    global collision_robot, collision_count, collision_start, collision_time
    global hasfood_robot, hasfood_count, hasfood_start, hasfood_time
    global prev_xpos, prev_ypos, distance
    global Food_moved, food_xpos, food_ypos, food_zpos,convergence_time_food, conv_start_time_food, New_Foodfound

    #decode the raw data
    #res = [","] * 10 #comas
    #res = re.findall(r"\"(.*?)\"", strs, re.DOTALL)[2:3] # time
    #res += re.findall(r"> (.*?);", strs, re.DOTALL) #2nd part
    #mylist = strs.split(" ->", 1)[0] 
    #mylist = mylist.replace(')',',')
    #mylist = mylist.replace(" ","")
    #res += mylist.split(',')[5:9]
    
    mylist = strs.split(" ->", 1)[0]
    mylist = mylist.replace('\'','')
    mylist = mylist.replace(')','')
    mylist = mylist.replace(" ","")
    mylist = mylist.replace('data:(','')
    res = mylist.split(',')
    res += re.findall(r"> (.*?)$", strs, re.DOTALL)



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
    except:
        pass
    try:
        foraging = res[6]
    except:
        foraging = ""
    try:
        foodcount = res[7]
    except:
        foodcount = ""
    try:
        hasfood = res[8]
    except:
        hasfood = ""
    try:
        collision = res[9]
    except:
        collision = ""
    try:
        action = res[10]
    except:
        action = ""

    #print time, robot, xpos, ypos, zpos, wpos, action, foraging, foodcount, hasfood

    #log experiment time
    try:
        if action == 'Starting Experiment':
            print "Starting Experiment"
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

    #log deliverd food for each robot
    try: 
        if action == 'Delivered_Food':
            foraging_count[current_robot] += 1
    except:
        pass

    #log deliverd food for each robot
    try: 
        if New_foodfound == False:
            if action == 'SearchFood':
                New_Foodfound = True
    except:
        pass


    # log foraging and convergence time and count per robot
    try:
        if foraging == 'True':
            if foraging_robot[current_robot] == False: #start new forage run(s)
                #foraging_count[current_robot] += 1
                foraging_robot[current_robot] =  True
                foraging_start[current_robot] = float(time)
                for robotnr in range(1,robots_in_experiment+1): # check if all robots are currently foraging
                    if foraging_robot[robotnr] != True:
                        convergence = False
                        Food_moved = False
                        break
                    else:
                        convergence = True
                if convergence == True and Food_moved == False:
                    convergence_start = float(time)
                    # if convergence_time_first == 0: # time till first convergence
                    # convergence_time_first = float(time) - starttime 
                    # if conv_start_time_food != starttime:
                    convergence_time_food[convergence_counter] = float(time) - conv_start_time_food
                    lfile.write("convergence to new food location in (sec) %s \n" % convergence_time_food[convergence_counter])
                    convergence_counter += 1

        else: 
            Food_moved = False
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
                #foraging_count[current_robot] += 1
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


    # log collision time and count per robot
    try:
        if robot == 'mitro':
            current_robot = 0
        if collision == 'True':
            if collision_robot[current_robot] == False: #new collision
                collision_count[current_robot] += 1
                collision_robot[current_robot] =  True
                collision_start[current_robot] = float(time)
        else: 
            if collision_robot[current_robot] == True: #collision resolved
                collision_robot[current_robot] =  False
                collision_time[current_robot] += float(time) - collision_start[current_robot]
    except:
        pass


    # log travel distance per robot
    try:
        if robot == 'mitro':
            current_robot = 0
        if distance[current_robot] == None: # init prev x-pos and ypos with startlocation for a new robot
            distance[current_robot] = 0
        else:
            distance[current_robot] += math.sqrt(math.pow((float(xpos) - prev_xpos[current_robot]),2) + math.pow((float(ypos) - prev_ypos[current_robot]),2))
        prev_xpos[current_robot] = float(xpos)
        prev_ypos[current_robot] = float(ypos)
    except:
        pass


    # log food position
    try:
        if robot == 'food':
            if (float(xpos)!= food_xpos) or (float(ypos) != food_ypos) or (float(zpos) != food_zpos):
                Food_moved = True
                food_xpos = float(xpos)
                food_ypos = float(ypos)
                food_zpos = float(zpos)
                lfile.write("FOOD moved to location %s %s %s %s \n" % (xpos, ypos, zpos, float(time)))
                conv_start_time_food = float(time)
    except:
        pass

# write column lables to file
ofile.write("time, robot, xpos, ypos, zpos', wpos, action, foraging, foodcount, hasfood, collision \n")


# iterate trough the file and decode
#rownum = 0
#for row in reversed(list(reader)):
    ## Save header row.
    #if rownum == 0:
        #header = row
    #else:
        #colnum = 0
        #for col in row:
            ##print '%-8s: %s' % (header[colnum], col)
        #if colnum == 0:
                #decode(col)
            #colnum += 1
    #rownum += 1
    
for line in reader:
    if "---" not in line:
        decode(line)
    


#adjust foraged time when still foraging at end of experiment
for robotnr in range(1,MaxRobots):
  if foraging_robot[robotnr] == True: #robot still foraging
     foraging_time[robotnr] += endtime - foraging_start[robotnr] 
     #print "correcting foraging time"
#adjust convergence time when still converged at end of experiment
if convergence == True: 
   convergence_time += endtime - convergence_start
   #print "correcting convergence time"

#total forage runs
for robotnr in range(1,MaxRobots):
  foraging_total += foraging_count[robotnr]
  hasfood_total += hasfood_count[robotnr]
  collision_total += collision_count[robotnr]

# calculate runtime of the experiment   

try:
  runtime = endtime - starttime
except:
  runtime = 0
try:
  troughput = (foraging_total /runtime)
except:
  troughput = 0

NrRobots = robots_in_experiment+1

#log output
lfile.write("\n****************************************************************************************\n")
lfile.write( "Decoding " + file_name + " --> %s \n" % outfile)
lfile.write( "General:              MaxRobots                       = %s \n" % MaxRobots )
lfile.write( "General:              Robots in experiment            = %s \n" % robots_in_experiment)
lfile.write( "General:              Runtime experiment (sec)        = %s \n" % runtime)
lfile.write( "****************************************************************************************\n")
lfile.write( "Sherpherding:         Times enabled                   = %s \n" % Shepherd_count)
lfile.write( "Sherpherding:         Total enabled time (sec)        = %s \n" %  Sheptotaltime)
lfile.write( "Barked:               Total amount                    = %s \n" % Barkcounter)
lfile.write( "Barked:               Times at Robots [1..NrRobots]   = %s \n" % BarkedAtRobot[1:NrRobots])
lfile.write( "Foodloc Asked:        Total times asked               = %s \n" % Askcounter)
lfile.write( "Foodloc Asked:        Times per Robot [1..NrRobots]   = %s \n" % AskedRobot[1:NrRobots])
lfile.write( "Foodloc Received:     Total times Received            = %s \n" % Receivecounter)
lfile.write( "Foodloc Received:     Times per Robot [1..NrRobots]   = %s \n" % Receivedfood[1:NrRobots])
lfile.write( "Foodloc Send:         Times per Robot [1..NrRobots]   = %s \n" % Sendfood[1:NrRobots])
lfile.write( "Foraging Runs:        Total                           = %s \n" % foraging_total)
lfile.write( "Foraging Runs:        per robots [1..NrRobots]        = %s \n" % foraging_count[1:NrRobots])
lfile.write( "Foraging Time:        per robot [1..NrRobots] (sec)   = %s \n" % foraging_time[1:NrRobots])
lfile.write( "Has food:             Total                           = %s \n" % hasfood_total)
lfile.write( "Has food:             per robots [1..NrRobots]        = %s \n" % hasfood_count[1:NrRobots])
lfile.write( "Has food Time:        per robot [1..NrRobots] (sec)   = %s \n" % hasfood_time[1:NrRobots])
lfile.write( "Dropped food:         Total                           = %s \n" % (hasfood_total - foraging_total))
lfile.write( "Dropped food:         per robots [1..NrRobots]        = %s \n" % map(sub,hasfood_count[1:NrRobots], foraging_count[1:NrRobots]))
lfile.write( "Collision:            Total                           = %s \n" % collision_total)
lfile.write( "Collision:            per robots [1..NrRobots]        = %s \n" % collision_count[1:NrRobots])
lfile.write( "Collision Time:       per robot [1..NrRobots] (sec)   = %s \n" % collision_time[1:NrRobots])
lfile.write( "Collision:            Mitro                           = %s \n" % collision_count[0])
lfile.write( "Collision Time:       Mitro (sec)                     = %s \n" % collision_time[0])
lfile.write( "Convergence:          Total time converged (sec)      = %s \n" % convergence_time)
lfile.write( "Convergence:          Time to first (sec)             = %s \n" % convergence_time_food[0])
lfile.write( "Convergence:          To Moved Food loc (sec)         = %s \n" % convergence_time_food[1:])
lfile.write( "Distance travelled:   per robots [1..NrRobots] (m)    = %s \n" % distance[1:NrRobots])
lfile.write( "Distance travelled:   mitro (m)                       = %s \n" % distance[0])
lfile.write( "****************************************************************************************\n")
lfile.write( "calc troughput:       food deliveries per second      = %s \n" %  troughput)
lfile.write( "calc troughput:       food deliveries per minute      = %s \n" %  (troughput*60))
lfile.write( "****************************************************************************************\n")



#close files
ifile.close()
ofile.close()
lfile.close()

#print log output to screen
with open(logfile, 'r') as fin:
    print fin.read()

