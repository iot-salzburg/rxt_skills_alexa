#! /usr/bin/env python

import paho.mqtt.client as paho
import rospy
import time

import actionlib # Brings in the SimpleActionClient
import rxt_skills_panda.msg # Brings in the messages used by the panda actions

received_msg = []

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
            global received_msg
            print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))  
            alexa_text = str(msg.payload)  
            received_msg.append(alexa_text)
            try:
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
                result = send_ROSActionRequest_WithGoal('PutObject', rxt_skills_panda.msg.GrabObjectAction, rxt_skills_panda.msg.GrabObjectGoal(objectPosition=str.encode(split_msg[1])))
                if result:
                    print("Result was: " + str(result.isOK))
                received_msg.pop(0)
                print ('----------------------------------')

        client = paho.Client()
        client.on_subscribe = on_subscribe
        client.on_message = on_message
        client.connect('80.158.53.59', 1883)
        client.subscribe('robxtask', qos=1)

        client.loop_forever()

     
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
