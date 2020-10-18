
from __future__ import print_function
import time
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, Float32
from mav_msgs.msg import RateThrust
class DroneController:
    def __init__(self, kp, kd):
        self.m_armed = False
        self.m_heightRef = None
        self.m_altitude = None
    
        self.m_Kp = kp
        self.m_Kd = kd

        rospy.init_node("drone_controller", anonymous=True)
        rospy.Subscriber("/uav/sensors/downward_laser_rangefinder", Range, self.altitudeCallback)
        rospy.Subscriber("/uav/height_reference", Float32, self.referenceHeightCallback)

        self.m_thrustPub = rospy.Publisher("/uav/input/rateThrust", RateThrust, queue_size=1)
        self.m_armPub = rospy.Publisher("/uav/input/arm", Empty, queue_size=1)

    def altitudeCallback(self, msg):
        self.m_altitude = msg.range 

        # if height reference is not set, height control is disabled
        if self.m_heightRef != None:
            thrust = RateThrust()
            thrust.thrust.z = self.m_Kp * (self.m_heightRef - abs(self.m_altitude))
            print(thrust.thrust.z)
            self.m_thrustPub.publish(thrust)

    def referenceHeightCallback(self, msg):
        self.m_heightRef = msg.data

        if not self.m_armed:    
            self.m_armPub.publish(Empty())
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    drone_controller = DroneController(45, 0)
    drone_controller.run()    