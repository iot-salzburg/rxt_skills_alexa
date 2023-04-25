#! /usr/bin/env python

import paho.mqtt.client as paho
import rospy
import time


import actionlib # Brings in the SimpleActionClient
import rxt_skills_panda.msg # Brings in the messages used by the panda actions

received_msg = []
split_msg = None

#------------------------------------------------------------------------------------------------------------------------------------------------------------
# client request helper function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
def send_ROSActionRequest_WithGoal(skillName, skillMsgType, skillGoal):

    rospy.init_node('alexa_client_py') # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS

    client = actionlib.SimpleActionClient(skillName, skillMsgType) # Creates SimpleActionClient with skillMsgType action type
    client.wait_for_server() # Waits until the action server has started up and started listening for goals
    client.send_goal(skillGoal) # Sends the goal to the action server
    client.wait_for_result() # Waits for the server to finish performing the action
    
    return client.get_result() # Prints out the result (WaitForUserInputResult) of executing the action
	
#------------------------------------------------------------------------------------------------------------------------------------------------------------
# main function
#------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        def on_subscribe(client, userdata, mid, granted_qos):
            print("Subscribed: "+str(mid)+" "+str(granted_qos))

        def on_message(client, userdata, msg):
            global received_msg, split_msg
            print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))  
            alexa_text = str(msg.payload.decode('utf-8'))  
            received_msg.append(alexa_text)
            try:
                print("Received data: ", received_msg[0])
                split_msg = received_msg[0].split('_')
                
            except:
                received_msg.pop(0)


            if split_msg[0] == 'pick':
                print ('----------------------------------')
                print ('INVOKING RXT_SKILL: GrabObject')
                result = send_ROSActionRequest_WithGoal('GrabObject', rxt_skills_panda.msg.GrabObjectAction, rxt_skills_panda.msg.GrabObjectGoal(objectPosition=str.encode(split_msg[1])))
                if result:
                    print("Result was: " + str(result.isOK))
                received_msg.pop(0)
                print ('----------------------------------')

            
            if split_msg[0] == 'place':
                print ('----------------------------------')
                print ('INVOKING RXT_SKILL: PutObject')
                result = send_ROSActionRequest_WithGoal('PutObject', rxt_skills_panda.msg.PutObjectAction, rxt_skills_panda.msg.PutObjectGoal(position=str.encode(split_msg[1])))
                if result:
                    print("Result was: " + str(result.isOK))
                
                received_msg.pop(0)
                print ('----------------------------------')


        client1 = paho.Client()
        client1.on_subscribe = on_subscribe
        client1.on_message = on_message
        client1.connect('80.158.53.59', 1883)
        client1.subscribe('robxtask', qos=0)

        client1.loop_forever()  # Start the loop in a separate thread

        try:
            while True:
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Program interrupted by user")
        finally:
            client1.loop_stop()  # Stop the loop
            client1.disconnect()  # Disconnect the client

     
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
