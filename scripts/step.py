    




    global pt
    #print(pt,'inside the step')
    
    GPIO.output(PUL_PIN, GPIO.HIGH)
    #t_k = time.time()
    #time.sleep(pt)  # Smallest delay possible for step pulse
    rospy.sleep(pt)
    #print("RST v DST", (time.time() - t_k), pt)
    GPIO.output(PUL_PIN, GPIO.LOW)
    rospy.sleep(pt)  # Smallest delay possible for step pulse
    #print("Step omega", 0.015708/(time.time() - t_k))