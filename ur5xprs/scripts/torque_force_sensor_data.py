#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import socket
from time import gmtime, strftime
import ConfigParser
import time
from gripper import *
from ur5xprs.srv import *
from std_msgs.msg import Bool
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

class socket_connection ():

    def __init__ (self, publisher_object, rate):

        try:
		
            config = ConfigParser.ConfigParser()
            config.read("config.ini")
	
            self.host_ip = config.get('Information', 'ip_address_ur5')
            self.port = int (config.get ('Information', 'port'))
            self.output_file_location = config.get ('Information', 'output_file_location') 
            
            self.write_object = True

            if self.output_file_location is "":
                self.output_file_location = ""
            elif self.output_file_location is "None":
                self.write_object = False
            else:
                self.output_file_location = self.output_file_location + "/"

                
        except:

            print ("Warning: Config.ini file problem, please check Config.ini")

        self.socket_object = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.publisher_object = publisher_object
        self.publisher_rate = rate
        self.is_published = False
	self.past_data = []
	self.base_data = []

    def connect (self):

        try:

            print("Warning: Connecting to Ur5 ip adress: " + self.host_ip)
            self.socket_object.connect(( self.host_ip, self.port ))

            
            if self.write_object is True:
                print("Warning: File Write Location: " + self.output_file_location)
                f = open (self.output_file_location + "DataStream.csv", "w")

            
                       
	    
            try:
                print("Writing in DataStream.csv, Press ctrl + c to stop")
		i = 1        
		rob = urx.Robot("10.0.0.2")
		robotiqGrip = Robotiq_Two_Finger_Gripper(rob)
		while (i):
		    change = 0		
                    received_data = self.socket_object.recv(1024)

                    received_data =  received_data.replace("(", "*").replace(")", "*")
		    value_list = received_data.split("*")
		    
                    received_data = value_list[1].split(" , ")
		    

		   

		    if '' in received_data:
		        received_data = received_data.remove('')
		    if self.base_data == []:
		    	self.base_data = [received_data[0],received_data[1],received_data[2]]
	##	    received_data[0] = received_data[0][0:7]
	##	    received_data[1] = received_data[1][0:7]
	##	    received_data[2] = received_data[2][0:7]
	##	    received_data[3] = received_data[3][0:7]
	##	    received_data[4] = received_data[4][0:7]
	##	    received_data[5] = received_data[5][0:7]
                    print(received_data)

		    if (i != 1 and received_data != [''] and self.base_data != [''] and received_data is not None and self.base_data is not None):
		    	change += abs(float(received_data[0])-float(self.base_data[0]))
			change += abs(float(received_data[1])-float(self.base_data[1]))
			change += abs(float(received_data[2])-float(self.base_data[2]))
			
			if(change > 10):
				robotiqGrip.gripper_action(100)
			
					
		    i += 1	
		    self.past_data = received_data
		    	
				
		    
		    """                    f_x = float(received_data.split(",")[0])
                    f_y = float(received_data.split(",")[1])
                    f_z = float(received_data.split(",")[2])
                    q_x = float(received_data.split(",")[3])
                    q_y = float(received_data.split(",")[4])
                    q_z = float(received_data.split(",")[5])
                   
                    denemeF = float(received_data.split(",")[2])
		    """
##                    force_torque_values = (self.socket_object.recv(1024).replace("(","*")).replace(")","*")
                    
##                    value_list = force_torque_values.split("*")
                    
##                    rospy.loginfo(value_list [1])
                    
##                    rospy.loginfo(f_x)
##                    rospy.loginfo(f_y)
##                    rospy.loginfo(f_z)
		    ##rospy.loginfo(received_data)

                    
##                    self.publisher_object.publish(value_list[1])
                    ##self.publisher_object.publish(denemeF)
                    self.publisher_rate.sleep()

##                    if self.write_object is True:       
##                        f.write(strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime()) + ": " +force_torque_values)
                                              
            except KeyboardInterrupt:
                        
                f.close
                self.socket_object.close



        except Exception as e:

            print("Error: No Connection!! Please check your ethernet cable :)" + str(e))





def main():
    

##    pub = rospy.Publisher('ft300_force_torque', String, queue_size=10)
    
    pub = rospy.Publisher('ft300_force_torque', Float32, queue_size=10)    
    rospy.init_node('torque_force_sensor_data', anonymous=True)
        
    rate = rospy.Rate(10) 

    socket_connection_obj = socket_connection(pub, rate)    
    socket_connection_obj.connect()
          

if __name__ == "__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        pass                         

    
                        
 	
