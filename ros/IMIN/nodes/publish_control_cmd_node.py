#!/usr/bin/env python
"""
reed to run roslaunch first, e.g.,

roslaunch bair_car bair_car.launch use_zed:=true record:=false
"""
import sys
import traceback
import runtime_parameters as rp
from pure_pursuit import pid_control, pure_pursuit_control
try:    
    ########################################################
    #          ROSPY SETUP SECTION
    import roslib
    import std_msgs.msg
    import geometry_msgs.msg
    import rospy

    rospy.init_node('run_AUTO',anonymous=True)

    state = 0
    previous_state = 0
    state_transition_time_s = 0

    
    def state_callback(data):
        global state, previous_state
        if state != data.data:
            if state in [3,5,6,7] and previous_state in [3,5,6,7]:
                pass
            else:
                previous_state = state
        state = data.data
        
    def state_transition_time_s_callback(data):
        global state_transition_time_s
        state_transition_time_s = data.data


    GPS2_lat = -999.99
    GPS2_long = -999.99
    GPS2_lat_orig = -999.99
    GPS2_long_orig = -999.99
    def GPS2_lat_callback(msg):
        global GPS2_lat
        GPS2_lat = msg.data
        
    def GPS2_long_callback(msg):
        global GPS2_long
        GPS2_long = msg.data

    def GPS2_lat_orig_callback(msg):
        global GPS2_lat_orig
        GPS2_lat_orig = msg.data    

    def GPS2_long_orig_callback(msg):
        global GPS2_long_orig
        GPS2_long_orig = msg.data   

    def GPS2_speed_callback(msg):
        global GPS2_speed
        GPS2_speed = msg.data

    def GPS2_long_callback(msg):
        global GPS2_angle
        GPS2_angle = msg.data

    camera_heading = 49.0

    freeze = False
    def gyro_callback(msg):
        global freeze, gyro
        gyro = msg
        #if np.abs(gyro.y) > gyro_freeze_threshold:
        #    freeze = True
        if np.sqrt(gyro.y**2+gyro.z**2) > rp.gyro_freeze_threshold:
            freeze = True
           
        
    def acc_callback(msg):
        global freeze, acc
        acc = msg
        if np.abs(acc.z) > rp.acc_freeze_threshold_z:
            freeze = True
        if acc.y < rp.acc_freeze_threshold_z_neg:
            freeze = True
        if np.abs(acc.x) > rp.acc_freeze_threshold_x:
            freeze = True
        #if np.abs(acc.y) > acc_freeze_threshold_y:
        #    freeze = True

        
    encoder_list = []
    def encoder_callback(msg):
        global encoder_list
        encoder_list.append(msg.data)
        if len(encoder_list) > 30:
            encoder_list = encoder_list[-30:]

    ##
    ########################################################

    import thread
    import time

    rospy.Subscriber('/bair_car/state', std_msgs.msg.Int32,state_callback)
    rospy.Subscriber('/bair_car/state_transition_time_s', std_msgs.msg.Int32, state_transition_time_s_callback)

    rospy.Subscriber('/bair_car/GPS2_lat', std_msgs.msg.Float32, callback=GPS2_lat_callback)
    rospy.Subscriber('/bair_car/GPS2_long', std_msgs.msg.Float32, callback=GPS2_long_callback)
    rospy.Subscriber('/bair_car/GPS2_lat_orig', std_msgs.msg.Float32, callback=GPS2_lat_orig_callback)
    rospy.Subscriber('/bair_car/GPS2_long_orig', std_msgs.msg.Float32, callback=GPS2_long_orig_callback)
    rospy.Subscriber('/bair_car/GPS2_speed', std_msgs.msg.Float32, callback=GPS2_speed_callback)
    rospy.Subscriber('/bair_car/GPS2_angle', std_msgs.msg.Float32, callback=GPS2_angle_callback)

    rospy.Subscriber('/bair_car/gyro', geometry_msgs.msg.Vector3, callback=gyro_callback)
    rospy.Subscriber('/bair_car/acc', geometry_msgs.msg.Vector3, callback=acc_callback)
    rospy.Subscriber('encoder', std_msgs.msg.Float32, callback=encoder_callback)

    steer_cmd_pub = rospy.Publisher('cmd/steer', std_msgs.msg.Int32, queue_size=100)
    motor_cmd_pub = rospy.Publisher('cmd/motor', std_msgs.msg.Int32, queue_size=100)
    freeze_cmd_pub = rospy.Publisher('cmd/freeze', std_msgs.msg.Int32, queue_size=100)
    model_name_pub = rospy.Publisher('/bair_car/model_name', std_msgs.msg.String, queue_size=10)

    ctr = 0


#============================================================================

    t0 = time.time()
    time_step = Timer(1)
    AUTO_enter_timer = Timer(2)
    folder_display_timer = Timer(30)
    finish_test_timer = Timer(600) # finish test in 10 mins
    AUTO_steer_previous = 49
    AUTO_motor_previous = 49
    #verbose = False
    
    # load reference way points
    # set initial condition
    cx = gps2x(way_points_GPS2_long)
    cy = gps2y(way_points_GPS2_lat)
    lastIndex = len(cx) - 1
    x = gps2x(GPS2_long)
    y = gps2y(GPS2_lat)
    yaw = gps2yaw(GPS2_angle)
    v = gps2v(GPS2_speed)
    state = State(x, y, yaw, v)
    target_ind = calc_target_index(state, cx, cy)
    target_speed = 10.0 / 3.6  # [m/s]
    
    while not rospy.is_shutdown():
        #state = 3 
        if state in [3,5,6,7]:
            
            if (previous_state not in [3,5,6,7]):
                previous_state = state
                AUTO_enter_timer.reset()
            if rp.use_PP:
                if not AUTO_enter_timer.check():
                    print "waiting before entering AUTO mode..."
                    steer_cmd_pub.publish(std_msgs.msg.Int32(49))
                    motor_cmd_pub.publish(std_msgs.msg.Int32(49))
                    time.sleep(0.1)
                    continue
                else:
                 
                    state = State(gps2x(GPS2_long), gps2y(GPS2_lat), gps2yaw(GPS2_angle), gps2v(GPS2_speed))
                    AUTO_motor = pid_control(target_speed, gps2v(GPS2_speed))
                    AUTO_steer = pure_pursuit_control(state, cx, cy, target_ind)
                    di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
            
                    if AUTO_motor > rp.motor_freeze_threshold \
                    and np.array(encoder_list[0:3]).mean() > 1 \
                    and np.array(encoder_list[-3:]).mean()<0.2 \
                    and state_transition_time_s > 1:
                        freeze = True

                    if lastIndex >= target_ind:
                        print(Goal!)
                        freeze = True

                    # avoid experiment too long
                    if finish_test_timer:
                        freeze = True

                    if freeze:
                        print "######### FREEZE ###########"
                        AUTO_steer = 49
                        AUTO_motor = 49

                    freeze_cmd_pub.publish(std_msgs.msg.Int32(freeze))  
                    steer_cmd_pub.publish(std_msgs.msg.Int32(AUTO_steer))              
                    motor_cmd_pub.publish(std_msgs.msg.Int32(AUTO_motor))

                    
                    if True: #verbose:
                        print("{},{},{},{}".format(AUTO_motor,AUTO_steer,rp.motor_gain,rp.steer_gain,state))
                        #print AI_motor,AI_steer,motor_gain,steer_gain,state

        else:
            AUTO_enter_timer.reset()
            if state == 4:
                freeze = False
            if state == 2:
                freeze = False
            if state == 1:
                freeze = False
            if state == 4 and state_transition_time_s > 30:
                print("Shutting down because in state 4 for 30+ s")
                #unix('sudo shutdown -h now')
        if time_step.check():
            print(d2s("In state",state,"for",state_transition_time_s,"seconds, previous_state =",previous_state))
            time_step.reset()
            if not folder_display_timer.check():
                print("*** Data foldername = "+rp.foldername+ '***')

except Exception as e:
    print("********** Exception ***********************",'red')
    traceback.print_exc(file=sys.stdout)
    rospy.signal_shutdown(d2s(e.message,e.args))

