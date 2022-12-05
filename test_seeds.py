import subprocess
for i in range(100):
  command = "roslaunch flightcontroller fly.launch seed:=" + str(i)
  process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
  output, error = process.communicate()
  if "GOAL REACHED!!! Final path was correct." in output:
    print("Success on " + str(i))
  else:
    print("Failure on " + str(i))
    