import subprocess

p = subprocess.Popen(['./external/ros2/ros2', 'bag', 'record', '--all'])
p.wait()
