import rclpy
from rclpy.node import Node

import polyscope as ps
import numpy as np
import igl
import pyquaternion as pyq
import os

from std_msgs.msg import String
import time
# needed for mocap data
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from commsmsgs.msg import Rpicommspub
from commsmsgs.msg import Logsetup
from commsmsgs.msg import Logctrlbools
from commsmsgs.msg import Guicontrols

# for checking other node states
from commsmsgs.msg import Bmnpub
from commsmsgs.msg import Brimpub
from commsmsgs.msg import Rbquadsimpub
from commsmsgs.msg import Rrimpub

class vis(Node):

    def __init__(self):
        super().__init__('vis')
        
        root_folder = os.getcwd()
        
        # ros2 stuff
        # node inputs: sim, drone pos and drone rm;
        self.drone_state_msg = Rpicommspub()
        self.drone_state_subscription = self.create_subscription(Rpicommspub, '/GC/out/rpicomms', self.drone_state_callback, 10)
        self.drone_state_subscription
        
        # logs state sub
        self.log_state_msg = Logctrlbools()
        self.log_state_subscription = self.create_subscription(Logctrlbools, '/GC/internal/logctrl', self.log_state_callback, 10)
        self.log_state_subscription
        
        # logs setup pub
        self.log_setup_pub = self.create_publisher(Logsetup, '/GC/internal/logsetup', 10)
        
        # gui controls pub
        self.gui_controls_pub = self.create_publisher(Guicontrols, '/GC/internal/guictrls', 10)
        
        # other node subs to check states
        self.sim_state_msg = Rbquadsimpub()
        self.sim_state_subscription = self.create_subscription(Rbquadsimpub, '/GC/out/rbquadsim', self.sim_state_callback, 10)
        self.sim_state_subscription
        
        self.rrim_state_msg = Rrimpub()
        self.rrim_state_subscription = self.create_subscription(Rrimpub, '/GC/out/prim', self.rrim_state_callback, 10)
        self.rrim_state_subscription
        
        self.bmn_state_msg = Bmnpub()
        self.bmn_state_subscription = self.create_subscription(Bmnpub, '/GC/out/bmn', self.bmn_state_callback, 10)
        self.bmn_state_subscription
        
        self.brim_state_msg = Brimpub()
        self.brim_state_subscription = self.create_subscription(Brimpub, '/GC/out/brim', self.brim_state_callback, 10)
        self.brim_state_subscription
        
        # class vars
        self.participant_id = 0
        self.experiment_id = 0
        self.mt = 0
        self.movement_type = "FM"
        self.rim_type = 0
        # logs open, logs close, logs clear, sim start, sim stop, sim reset, bmn start, bmn stop, bmn reset, rpicomms start, rpicomms stop, rpicomms reset, irl drone takeoff, irl drone land
        self.controls = [False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        # logs running, logs stopped, bmn running, bmn stopped, brim running, brim stopped, sim running, sim stopped, rrim running, rrim stopped, rpicomms running, rpicomms stopped, irl flying, irl landed
        self.states = [False, True, False, True, False, True, False, True, False, True, False, True, False, False]
        self.sim_Drone_pos = [0, 0, 0]
        self.sim_drone_rm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.real_Drone_pos = [0, 0, 0]
        self.real_drone_rm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
        # Initialize the polyscope window
        ps.set_autocenter_structures(True)
        ps.set_build_gui(False)
        ps.set_window_size(3500, 2200)
        ps.set_open_imgui_window_for_user_callback(True)

        ps.init()

        ps.look_at([-2.0, 1.5, -15.0], [0, 0, 0])

        # Load a mesh
        self.RoomV, self.RoomF = igl.read_triangle_mesh(os.path.join(root_folder, "Haptic-RW-UAS-Ground-Controller/src/visualization_inputs/resource", "Room_Large_Zn.obj"))
        self.DroneV, self.DroneF = igl.read_triangle_mesh(os.path.join(root_folder, "Haptic-RW-UAS-Ground-Controller/src/visualization_inputs/resource", "Holybro_np.obj"))
        self.RDroneV, self.RDroneF = igl.read_triangle_mesh(os.path.join(root_folder, "Haptic-RW-UAS-Ground-Controller/src/visualization_inputs/resource", "Holybro_np.obj"))

        self.DroneVOG = self.DroneV
        drone_rot_swap = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        self.DroneVOG_Transpose = np.transpose(self.DroneVOG)
        self.DroneV = np.transpose(np.dot(drone_rot_swap, self.DroneVOG_Transpose))
        self.DroneVOG = self.DroneV
        
        self.RDroneVOG = self.RDroneV
        self.RDroneVOG_Transpose = np.transpose(self.RDroneVOG)
        self.RDroneV = np.transpose(np.dot(drone_rot_swap, self.RDroneVOG_Transpose))
        self.RDroneVOG = self.RDroneV

        # Register the mesh with polyscope
        Drone_sur = ps.register_surface_mesh("Drone", self.DroneV, self.DroneF)
        Real_Drone_sur = ps.register_surface_mesh("Real_Drone", self.RDroneV, self.RDroneF)
        Room_sur = ps.register_surface_mesh("Room", self.RoomV, self.RoomF)

        # you can also access the structure by name
        ps.get_surface_mesh("Drone").set_color([0.2, 0.2, 0.2])
        ps.get_surface_mesh("Real_Drone").set_color([0.5, 0.1, 0.1])
        ps.get_surface_mesh("Room").set_color([0.8, 0.8, 0.8])

        ps.set_user_callback(self.vis_callback)

        # Show the GUI
        ps.show()

    # handle the mocap data
    def drone_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.drone_state_msg = msg
        self.states[12] = self.drone_state_msg.drone_is_flying
        self.states[13] = self.drone_state_msg.drone_is_landed
        self.real_Drone_pos = [self.drone_state_msg.actual_drone_position.x, self.drone_state_msg.actual_drone_position.y, self.drone_state_msg.actual_drone_position.z]
        real_Drone_orientation = pyq.Quaternion(self.drone_state_msg.actual_drone_orientation.w, self.drone_state_msg.actual_drone_orientation.x, self.drone_state_msg.actual_drone_orientation.y, self.drone_state_msg.actual_drone_orientation.z)
        self.real_drone_rm = real_Drone_orientation.rotation_matrix
        if self.drone_state_msg.running:
            self.states[10] = True
            self.states[11] = False
        else:
            self.states[10] = False
            self.states[11] = True
        
    # handle the log state data
    def log_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.log_state_msg = msg
        self.states[0] = self.log_state_msg.opened
        self.states[1] = self.log_state_msg.closed
        
    # handle the rrim state data
    def rrim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.rrim_state_msg = msg
        if self.rrim_state_msg.running:
            self.states[8] = True
            self.states[9] = False
        else:
            self.states[8] = False
            self.states[9] = True
        
    # handle the sim state data
    def sim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.sim_state_msg = msg
        self.sim_Drone_pos = [self.sim_state_msg.position.x, self.sim_state_msg.position.y, self.sim_state_msg.position.z]
        self.sim_drone_rm = self.sim_state_msg.rotation_matrix
        if self.sim_state_msg.running:
            self.states[6] = True
            self.states[7] = False
        else:
            self.states[6] = False
            self.states[7] = True
        
    # handle the brim state data
    def brim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.brim_state_msg = msg
        if self.brim_state_msg.running:
            self.states[4] = True
            self.states[5] = False
        else:
            self.states[4] = False
            self.states[5] = True
        
    # handle the bmn state data
    def bmn_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.bmn_state_msg = msg
        if self.bmn_state_msg.running:
            self.states[2] = True
            self.states[3] = False
        else:
            self.states[2] = False
            self.states[3] = True
        
    # publish gui controls commands
    def gui_controls_pub_callback(self):
        # publish the gui controls message
        msg = Guicontrols()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.open_logs = self.controls[0]
        msg.close_logs = self.controls[1]
        msg.clear_logs = self.controls[2]
        msg.start_simulation = self.controls[3]
        msg.stop_simulation = self.controls[4]
        msg.reset_simulation = self.controls[5]
        msg.start_bmn = self.controls[6]
        msg.stop_bmn = self.controls[7]
        msg.reset_bmn = self.controls[8]
        msg.start_rpicomms = self.controls[9]
        msg.stop_rpicomms = self.controls[10]
        msg.reset_rpicomms = self.controls[11]
        msg.takeoff = self.controls[12]
        msg.land = self.controls[13]
        self.gui_controls_pub.publish(msg)
            
                   
    def vis_callback(self):
        # Create a window called "Control Panel" and place the widgets below it
        ps.imgui.Begin("Control Panel", True)
        ps.imgui.PushItemWidth(100)
        ps.imgui.SetWindowFontScale(1.5)
        ps.imgui.SetWindowPos([10, 10])
        
        # participant id iput box
        changed_pi, self.participant_id = ps.imgui.InputInt("Participant Number", self.participant_id)
        # experiment id input box
        changed_ei, self.experiment_id = ps.imgui.InputInt("Experiment Number", self.experiment_id)
        # test direction select menu
        changed_m, self.mt= ps.imgui.Combo("Movement Type", self.mt, ["Free Movement", "Guided Movement"])
        if self.mt == 0:
            self.movement_type = "FM"
        else:
            self.movement_type = "GM"
            
        # rim type select menu
        changed_r, self.rim_type = ps.imgui.Combo("RIM Type", self.rim_type, ["RIM0", "RIM1", "RIM2", "RIM3"])
        
        # logs controls, first thing to activate and last to close
        #self.get_logger().info("control commands sent open_logs: %d close_logs: %d clear_logs: %d start sim: %d stop sim: %d reset sim: %d start bmn: %d stop bmn: %d reset bmn: %d start rpicomms: %d stop rpicomms: %d reset rpicomms: %d takeoff: %d land: %d" % (self.controls[0], self.controls[1], self.controls[2], self.controls[3], self.controls[4], self.controls[5], self.controls[6], self.controls[7], self.controls[8], self.controls[9], self.controls[10], self.controls[11], self.controls[12], self.controls[13]))
        #self.get_logger().info("state values logs running: %d logs stopped: %d bmn running: %d bmn stopped: %d brim running: %d brim stopped: %d sim running: %d sim stopped: %d rrim running: %d rrim stopped: %d rpicomms running: %d rpicomms stopped: %d irl flying: %d irl landed: %d" % (self.states[0], self.states[1], self.states[2], self.states[3], self.states[4], self.states[5], self.states[6], self.states[7], self.states[8], self.states[9], self.states[10], self.states[11], self.states[12], self.states[13]))
        # name logs button
        # if button pressed and logs are currently closed
        if (ps.imgui.Button("Name Logs") and self.states[1]):
            self.get_logger().info('Naming Logs')
            # publish the log setup message
            msg = Logsetup()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.participant_id = self.participant_id
            msg.experiment_id = self.experiment_id
            msg.movement_type = self.movement_type
            msg.rim_type = self.rim_type
            self.log_setup_pub.publish(msg)
            
        # open logs button
        # if button pressed and logs are currently closed
        if (ps.imgui.Button("Open Logs") and self.states[1]):
            self.get_logger().info('Opening Logs')
            self.controls[0] = True
            self.controls[1] = False
            self.controls[2] = False
            
        # close logs button
        # if button pressed and logs are currently open
        if (ps.imgui.Button("Close Logs") and self.states[0]):
            self.get_logger().info('Closing Logs')
            self.controls[0] = False
            self.controls[1] = True
            self.controls[2] = False        
        
        # sim controls, first after logs to activate, last before logs to close
        # start simulation button
        # if button pressed and sim is currently closed and logs are open already
        # self.states = [False, True, False, True, False, True, False, True, False, True, False, True, False, False]
        #if (ps.imgui.Button("Start Simulation") and self.states[7] and self.states[0]):
        if (ps.imgui.Button("Start Simulation")):
            self.get_logger().info('Starting Simulation')
            self.controls[3] = True
            self.controls[4] = False
            self.controls[5] = False
        
        # stop simulation button
        # if button pressed and sim is currently open and bmn/brim is closed and rpicomms/rrim is closed
        #if (ps.imgui.Button("Stop Simulation") and self.states[6] and self.states[5] and self.states[3] and self.states[11] and self.states[9]):
        if (ps.imgui.Button("Stop Simulation")):
            self.get_logger().info('Stopping Simulation')
            self.controls[3] = False
            self.controls[4] = True
            self.controls[5] = False            
            
        # bmn and brim controls, have to be activated after sim, stopped before sim, open bmn first and close brim first
        # start bmn button
        # if button pressed and bmn/brim is closed and sim is started already
        #if (ps.imgui.Button("Start BMN") and self.states[5] and self.states[3] and self.states[6]):
        if (ps.imgui.Button("Start BMN")):
            self.get_logger().info('Starting BMN')
            self.controls[6] = True
            self.controls[7] = False
            self.controls[8] = False
            
        # stop bmn button
        # if button pressed and bmn/brim is open
        #if (ps.imgui.Button("Stop BMN") and self.states[4] and self.states[2]):
        if (ps.imgui.Button("Stop BMN")):
            self.get_logger().info('Stopping BMN')
            self.controls[6] = False
            self.controls[7] = True
            self.controls[8] = False
            
        # rpicomms and rrim controls, have to activate after sim, stopped before sim, open rpicomms first and close rrim first
        # start rpicomms button
        # if button pressed and rpicomms/rrim is closed and sim is started already
        #if (ps.imgui.Button("Start Rpicomms") and self.states[11] and self.states[9] and self.states[6]):
        if (ps.imgui.Button("Start Rpicomms")):
            self.get_logger().info('Starting Rpicomms')
            self.controls[9] = True
            self.controls[10] = False
            self.controls[11] = False
            
        # stop rpicomms button
        # if button pressed and rpicomms/rrim is open
        #if (ps.imgui.Button("Stop Rpicomms") and self.states[10] and self.states[8]):
        if (ps.imgui.Button("Stop Rpicomms")):
            self.get_logger().info('Stopping Rpicomms')
            self.controls[9] = False
            self.controls[10] = True
            self.controls[11] = False        
        
        ps.imgui.PopItemWidth()
        ps.imgui.End()
        
        self.gui_controls_pub_callback()
        
        # update the drone objects position and orientation using the info from the ros2 msgs
        drows = self.DroneVOG.shape[0]
        d = np.ones((drows))
        DroneVnew = np.zeros((drows, 3))
        DroneVnew[:, 0] = -self.sim_Drone_pos[0] * d
        DroneVnew[:, 1] = (self.sim_Drone_pos[2]-5) * d
        DroneVnew[:, 2] = self.sim_Drone_pos[1] * d
        self.DroneV = np.transpose(np.dot(self.sim_drone_rm, np.transpose(self.DroneVOG))) + (1000 * DroneVnew)
        ps.get_surface_mesh("Drone").update_vertex_positions(self.DroneV)
        
        rdrows = self.RDroneVOG.shape[0]
        rd = np.ones((rdrows))
        RDroneVnew = np.zeros((rdrows, 3))
        RDroneVnew[:, 0] = -self.real_Drone_pos[0] * rd
        RDroneVnew[:, 1] = (self.real_Drone_pos[2]-5) * rd
        RDroneVnew[:, 2] = self.real_Drone_pos[1] * rd
        self.RDroneV = np.transpose(np.dot(self.real_drone_rm, np.transpose(self.RDroneVOG))) + (1000 * RDroneVnew)
        ps.get_surface_mesh("Real_Drone").update_vertex_positions(self.RDroneV)
        
    
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