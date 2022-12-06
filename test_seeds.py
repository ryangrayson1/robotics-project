import subprocess
import re
import threading
import os
import signal
import time
from random import randint

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
for h in range(23, 10, -4):
  for w in range(23, 10, -4):
    for s in [randint(1, 1000) for _ in range(2)]:
      print("running " + "width = " + str(w) + ", height = " + str(h) + ", seed = " + str(s))
      command = "roslaunch flightcontroller fly.launch seed:=" + str(s) + " map_width:=" + str(w) + " map_height:=" + str(h)
      try:
        process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        if "GOAL REACHED!!! Final path was correct." in output:
          time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
          print("Success on " + "w = " + str(w) + " h = " + str(h) + " s = " + str(s) + " | Time: " + str(time))
        else:
          print("Failure on " + "w = " + str(w) + " h = " + str(h) + " s = " + str(s))
      except Exception as e:
        print("Error on " + "w = " + str(w) + " h = " + str(h) + " s = " + str(s))
        print(e)
        continue
      