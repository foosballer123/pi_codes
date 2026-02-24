# The rosnode launched in this shell script is attached to this shell
# Unless the rosnode is detached it cannot run independetly from the shell that spawns it

echo "Launching motor3_test script..."
rosrun pi_codes motor3_test.py &        # The & sign spawns a background process so that the shell can continue to another command line
sleep 3


