import rclpy
from rclpy.node import Node

import polyscope as ps
import numpy as np
import igl

from std_msgs.msg import String
import time
# needed for mocap data
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point


class vis(Node):

    def __init__(self):
        super().__init__('vis')
        
        # class vars
        self.test_num = 0
        self.direction = 0
        self.Drone_pos = [0, 0, 0]
        self.drone_rm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
        # Initialize the polyscope window
        ps.set_autocenter_structures(True)
        ps.set_build_gui(False)
        ps.set_window_size(3500, 2200)
        ps.set_open_imgui_window_for_user_callback(True)

        ps.init()

        ps.look_at([-2.0, 1.5, -15.0], [0, 0, 0])

        # Load a mesh
        self.DroneV, _, _, self.DroneF, _, _= igl.read_obj("../resource/Holybro_np.obj")
        self.RoomV, _, _, self.RoomF, _, _= igl.read_obj("../resource/Room_Large_Zn.obj")

        self.DroneVOG = self.DroneV
        drone_rot_swap = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        self.DroneVOG_Transpose = np.transpose(self.DroneVOG)
        self.DroneV = np.transpose(np.dot(drone_rot_swap, self.DroneVOG_Transpose))
        self.DroneVOG = self.DroneV

        # Register the mesh with polyscope
        Drone_sur = ps.register_surface_mesh("Drone", self.DroneV, self.DroneF)
        Room_sur = ps.register_surface_mesh("Room", self.RoomV, self.RoomF)

        # you can also access the structure by name
        ps.get_surface_mesh("Drone").set_color([0.2, 0.2, 0.2])
        ps.get_surface_mesh("Room").set_color([0.8, 0.8, 0.8])

        ps.set_user_callback(self.vis_callback)

        # Show the GUI
        ps.show() 
        
        
        # ros2 stuff
        # node inputs: sim, drone pos and drone rm;
        # node outputs: test_num, direction, other user inputs
        
        # reference ros2 stuff
        # class vars
        self.mocapoptimsg = PoseStamped()
        # create mocap system subscriber
        self.qos = rclpy.qos.QoSProfile(depth=5,reliability=rclpy.qos.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,avoid_ros_namespace_conventions=False)
        self.opti_subscription = self.create_subscription(PoseStamped, '/vrpn_mocap/baby_michael/pose', self.mocap_callback, self.qos)
        self.opti_subscription

        ob_timer_period = 0.01 #seconds
        self.offboardtimer = self.create_timer(ob_timer_period, self.offboard_timer_callback)
        self.offboard_setpoint_counter = 0
        
        # ground control computer comms publishing current drone state and subbing to external setpoint
        self.ext_setpoint = {0, 0, 0}
        self.ext_cmd_subscription = self.create_subscription(Point, '/rpi/in/ext_cmdPoint', self.ext_cmd_callback, 10)
        self.gcc_local_position_pub = self.create_publisher(Point, '/rpi/out/localPoint', 10)
        self.ground_control_timer = self.create_timer(ob_timer_period, self.gcc_timer_callback)

    # handle the mocap data
    def mocap_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        self.mocapoptimsg = msg
        self.px4msg.position = [self.optimsg.pose.position.x, self.optimsg.pose.position.z, -1*self.optimsg.pose.position.y]
        self.px4msg.q = [self.optimsg.pose.orientation.x, self.optimsg.pose.orientation.z, -1*self.optimsg.pose.orientation.y, self.optimsg.pose.orientation.w]        
        
    # handle the external setpoint
    def ext_cmd_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        self.ext_setpoint = {msg.x, msg.y, msg.z}
            
    # publish the current state of the drone to the ground control computer
    def gcc_timer_callback(self):
        msg = Point()
        msg.x = self.statepx4msg.x
        msg.y = self.statepx4msg.y
        msg.z = self.statepx4msg.z
        self.gcc_local_position_pub.publish(msg)
        self.get_logger().info('Sending to gcc: "%s"' % msg)
       
        
    def vis_callback(self):
        # Create a window called "Control Panel" and place the widgets below it
        ps.imgui.Begin("Control Panel", True)
        ps.imgui.PushItemWidth(100)
        ps.imgui.SetWindowFontScale(1.5)
        ps.imgui.SetWindowPos([10, 10])
        
        # test number iput box
        changed_tn, self.test_num = ps.imgui.InputInt("Test Number", self.test_num)
        # test direction select menu
        changed_td, self.direction = ps.imgui.Combo("Test Direction", self.direction, ["X", "Y", "Z"])
        
        # start test/sim button using above id number/ selections
        if ps.imgui.Button("Start Test/Sim"):
            print("Test Number: ", self.test_num)
            print("Test Direction: ", self.direction)
        
        # reset button
        if ps.imgui.Button("Reset"):
            print("Reset")
            
        # save logs button
        if ps.imgui.Button("Save Logs"):
            print("Save Logs")
            
        # overwrite logs button
        if ps.imgui.Button("Overwrite Logs"):
            print("Overwrite Logs")
            
        # stop test/sim button
        if ps.imgui.Button("Stop Test/Sim"):
            print("Stop Test/Sim")
        
        ps.imgui.PopItemWidth()
        ps.imgui.End()
        
        # update the drone objects position and orientation using the info from the ros2 msgs
        drows = self.DroneVOG.shape[0]
        d = np.ones((drows))
        DroneVnew = np.zeros((drows, 3))
        DroneVnew[:, 0] = -self.Drone_pos[0] * d
        DroneVnew[:, 1] = (self.Drone_pos[2]-5) * d
        DroneVnew[:, 2] = self.Drone_pos[1] * d
        self.DroneV = np.transpose(np.dot(self.drone_rm, np.transpose(self.DroneVOG))) + (1000 * DroneVnew)
        ps.get_surface_mesh("Drone").update_vertex_positions(self.DroneV)
        
    
# spins up and down nodes
def main(args=None):
    rclpy.init(args=args)

    visuals = vis()

    rclpy.spin(visuals)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visuals.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()