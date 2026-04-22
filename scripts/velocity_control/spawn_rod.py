#!/usr/bin/env python3

# This script:
# 1) Takes rod # as terminal passed argument
# 2) Runs a ros node for the linear motion of the rod where the motor # is calculated from the rod # by [(rod #)*2 - 1]
# 3) Runs a ros node for the angular motion of the rod where the motor # is calculated from the rod # by [(rod #)*2]
# 4) Safely kills both processes on close through SIGINT commands
# Note: Linear motors are odd #'s and angular motors are even #'s

import subprocess
import signal
import sys
import argparse
import rospy

parser = argparse.ArgumentParser(
        description="'Spawn rod' script that initializes the linear and angular motors for a single rod starting from 'rod 1' through 'rod n'."
)
parser.add_argument("--rod", required=True, type=int)

args = parser.parse_args(rospy.myargv()[1:])

rod = args.rod

processes = []

# Start motor nodes
def start_nodes():
    global processes
    
    # Start linear motor (2n-1) node
    p1 = subprocess.Popen([
        "rosrun", "pi_codes", "velocity_ctrl.py", "--motor="+str((rod*2)-1)
    ])
    
    # Start angular motor (2n) node
    p2 = subprocess.Popen([
        "rosrun", "pi_codes", "velocity_ctrl.py", "--motor="+str(rod*2)
    ])
    
    processes.extend([p1, p2])

# Cleanly shutdown motor nodes
def shutdown_nodes(signum=None, frame=None):
    print("\nShutting down nodes...")
    
    for p in processes:
        p.terminate()  # sends SIGTERM
    
    for p in processes:
        p.wait()
    
    print("All nodes shut down.")
    sys.exit(0)


if __name__ == "__main__":
    # Catch Ctrl+C
    signal.signal(signal.SIGINT, shutdown_nodes)
    
    start_nodes()
    
    # Keep script alive
    signal.pause()