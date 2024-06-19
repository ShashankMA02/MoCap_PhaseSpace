import os
import rospy

class MocapDataLogger:
    # Existing code...

    def run(self):
        rospy.init_node("mocap_data_logger")
        self.last_save_time = rospy.Time.now()  # Initialize last save time
        rate = rospy.Rate(self.save_rate)

        # Delete the existing CSV file if it exists
        if os.path.exists(self.mocap_data.csv):
            os.remove(self.mocap_data.csv)

        while not rospy.is_shutdown():
            self.save_to_csv()
            rate.sleep()
