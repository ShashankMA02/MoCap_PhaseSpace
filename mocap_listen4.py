''' This file is - Recording data at 1Hz and storing it in csv file 
    1. focas-main@focaslab-01:~$ roslaunch phasespace_bringup phasespace_mocap.launch (terminal-1 command)
    2. focas-main@focaslab-01:~/catkin_hexa_ws/Mocap_listener$ ./mocap_listen4.py  ( terminal-2 comand) '''



import os
import rospy
import csv
from phasespace_msgs.msg import Rigids  # Replace with the actual message type

class MocapDataLogger:
    def __init__(self):
        rospy.init_node("mocap_data_logger")  # Initialize ROS node
        self.csv_filename = "mocap_data.csv"
        self.pose_data = None  # Store the most recent pose data
        self.pose_subscriber = rospy.Subscriber('/phasespace/rigids', Rigids, self.mocap_callback)
        self.record_count = 0  # Variable to count the number of records saved

    def mocap_callback(self, msg):
        # Extract position and orientation data for the rigid body with id: 1
        for rigid_body in msg.rigids:
            if rigid_body.id == 1:
                self.pose_data = [rigid_body.x, rigid_body.y, rigid_body.z,
                                  rigid_body.qx, rigid_body.qy, rigid_body.qz, rigid_body.qw]
                print("Received pose data for rigid body 1:", self.pose_data)  # Debugging statement
                break  # Assuming we only care about the first occurrence of id: 1

    def save_to_csv(self):
        if self.pose_data:
            with open(self.csv_filename, mode='a') as file:
                writer = csv.writer(file)
                writer.writerow(self.pose_data)
            self.record_count += 1
            print("Data recorded at:", rospy.Time.now(), "Total records:", self.record_count)

    def run(self):
        rate = rospy.Rate(1)  # Set rate to 1 Hz
        while not rospy.is_shutdown():
            self.save_to_csv()
            rate.sleep()


if __name__ == "__main__":
    mocap_logger = MocapDataLogger()
    mocap_logger.run()
