import subprocess
import re
for i in range(2, 1000):
  command = "roslaunch flightcontroller fly.launch seed:=" + str(i)
  process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
  output, error = process.communicate()
  if "GOAL REACHED!!! Final path was correct." in output:
    time = float(re.search("Time taken: ([0-9.]*) seconds", output).group(1))
    print("Success on " + str(i) + " | Time: " + str(time))
  else:
    print("Failure on " + str(i))
    