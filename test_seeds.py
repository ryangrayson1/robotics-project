import subprocess
import re
import threading
import os
import signal
import time

# print("-------------------- 11x11 grids --------------------")
# for i in range(100):
#   command = "roslaunch flightcontroller fly.launch seed:=" + str(i) + " map_width:=11 map_height:=11"
#   process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
#   output, error = process.communicate()

#   if "GOAL REACHED!!! Final path was correct." in output:
#     time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
#     print("Success on " + str(i) + " | Time: " + str(time))
#   else:
#     print("Failure on " + str(i))
    
# print("-------------------- 11x23 grids --------------------")
# for i in range(100):
#   command = "roslaunch flightcontroller fly.launch seed:=" + str(i) + " map_width:=11 map_height:=23"
#   process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
#   output, error = process.communicate()
#   if "GOAL REACHED!!! Final path was correct." in output:
#     time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
#     print("Success on " + str(i) + " | Time: " + str(time))
#   else:
#     print("Failure on " + str(i))

# print("-------------------- 23x11 grids --------------------")
# for i in range(100):
#   command = "roslaunch flightcontroller fly.launch seed:=" + str(i) + " map_width:=23 map_height:=11"
#   process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
#   output, error = process.communicate()
#   if "GOAL REACHED!!! Final path was correct." in output:
#     time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
#     print("Success on " + str(i) + " | Time: " + str(time))
#   else:
#     print("Failure on " + str(i))

# print("-------------------- 23x23 grids --------------------")
# for i in range(100):
#   command = "roslaunch flightcontroller fly.launch seed:=" + str(i) + " map_width:=23 map_height:=23"
#   process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
#   output, error = process.communicate()
#   if "GOAL REACHED!!! Final path was correct." in output:
#     time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
#     print("Success on " + str(i) + " | Time: " + str(time))
#   else:
#     print("Failure on " + str(i))
  
print("-------------------- varied size grids --------------------")
for w in range(11, 35, 4):
  for h in range(11, 35, 4):
    for s in range(3, 300, 50):
      command = "roslaunch flightcontroller fly.launch seed:=" + str(s) + " map_width:=" + str(w) + " map_height:=" + str(h)
      process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
      output, error = process.communicate()
      if "GOAL REACHED!!! Final path was correct." in output:
        time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
        print("Success on " + str(s) + " | Time: " + str(time))
      else:
        print("Failure on " + str(s))