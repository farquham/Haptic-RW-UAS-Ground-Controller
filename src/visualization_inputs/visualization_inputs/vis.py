import rclpy
from rclpy.node import Node

import polyscope as ps
import numpy as np
import igl
import pyquaternion as pyq
import os
import multiprocessing as mp
import random

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
    _participant_id = mp.Value('i', 0)
    _experiment_id = mp.Value('i', 0)
    _movement_type = mp.Value('i', 0)
    _rim_type = mp.Value('i', 0)
    _controls = mp.Array('b', 14)
    _states = mp.Array('b', 14)
    _sim_drone_pos = mp.Array('d', 3)
    _sim_drone_rm = mp.Array('d', 9)
    _real_drone_pos = mp.Array('d', 3)
    _real_drone_rm = mp.Array('d', 9)
    _update = mp.Value('b', 0)
    _setup_logs = mp.Value('b', 0)
    
    def __init__(self):
        super().__init__('vis')        
        # ros2 stuff
        # node inputs: sim, drone pos and drone rm;
        self.drone_state_msg = Rpicommspub()
        self.drone_state_subscription = self.create_subscription(Rpicommspub, '/GC/out/rpicomms', self.drone_state_callback, 10)
        self.drone_state_subscription
        
        # logs state sub
        self.log_state_msg = Logctrlbools()
        self.log_state_subscription = self.create_subscription(Logctrlbools, '/GC/internal/logctrl', self.log_state_callback, 10)
        self.log_state_subscription
        
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
        
        # logs setup pub
        self.log_setup_pub = self.create_publisher(Logsetup, '/GC/internal/logsetup', 10)
        self.log_setup_timer = self.create_timer(0.1, self.log_setup_pub_callback)
        
        # gui controls pub
        self.gui_controls_pub = self.create_publisher(Guicontrols, '/GC/internal/guictrls', 10)
        self.gui_controls_timer = self.create_timer(0.01, self.gui_callback)    
        
        # class vars
        self._participant_id.value = 0
        self._experiment_id.value = 0
        self._movement_type.value = 0
        self._rim_type.value = 0
        # logs open, logs close, logs clear, sim start, sim stop, sim reset, bmn start, bmn stop, bmn reset, rpicomms start, rpicomms stop, rpicomms reset, irl drone takeoff, irl drone land
        self._controls[:] = [False, False, False, False, False, False, False, False, False, False, False, False, False, False]
        # logs running, logs stopped, bmn running, bmn stopped, brim running, brim stopped, sim running, sim stopped, rrim running, rrim stopped, rpicomms running, rpicomms stopped, irl flying, irl landed
        self._states[:] = [False, True, False, True, False, True, False, True, False, True, False, True, False, False]
        self._sim_drone_pos[:] = [0.0, 0.0, 0.0]
        self._sim_drone_rm[:] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self._real_drone_pos[:] = [0.0, 0.0, 0.0]
        self._real_drone_rm[:] = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        self._cmd_drone_pos = [0.0, 0.0, 0.0]
        self._update.value = 0
        self._setup_logs.value = 0
        
        self._process = mp.Process(
            target=visualizations,
            args=(self._participant_id, 
                  self._experiment_id,
                  self._movement_type, 
                  self._rim_type, 
                  self._controls, 
                  self._states, 
                  self._sim_drone_pos, 
                  self._sim_drone_rm, 
                  self._real_drone_pos, 
                  self._real_drone_rm,
                  self._cmd_drone_pos,
                  self._update,
                  self._setup_logs,
                  self.get_logger()
            )
        )
        self._process.start()
        
    def log_setup_pub_callback(self):
        if self._setup_logs.value:
            msg = Logsetup()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.participant_id = self._participant_id.value
            msg.experiment_id = self._experiment_id.value
            if self._movement_type.value == 0:
                msg.movement_type = "GMT"
            elif self._movement_type.value == 1:
                msg.movement_type = "CT"
            msg.rim_type = self.cosim_randomize(self._participant_id.value, self._movement_type.value, self._rim_type.value)
            self.log_setup_pub.publish(msg)
            self._setup_logs.value = 0
            
    def cosim_randomize(self, part_id, move_type, rim_type):
        # if rim option selected don't randomize as this run is for dev purposes
        if rim_type == 3:
            out = 2
        # if any of the other three options are selected pseudo randomize the output according to part_id, move_type, and rim_type
        # must always output the same result for a given set of inputs
        else:
            # pseudo randomize the output
            norm_1 = part_id % 3
            norm_2 = move_type
            norm_3 = rim_type % 3
            random.seed(part_id)
            randomness = random.randint(0, 2)
            out = (norm_1 + norm_2 + norm_3 + randomness) % 3      
            
        # out must be 0 ZOH, 1 FOH, or 2 RIM
        return out
            
    def gui_callback(self):
        if self._update.value:
            self.gui_controls_pub_callback()
            self._update.value = 0

    # handle the mocap data
    def drone_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.drone_state_msg = msg
        self._states[12] = self.drone_state_msg.drone_is_flying
        self._states[13] = self.drone_state_msg.drone_is_landed
        self._real_drone_pos[:] = [self.drone_state_msg.actual_drone_position.x, self.drone_state_msg.actual_drone_position.y, self.drone_state_msg.actual_drone_position.z]
        # reorganize quats to match visualization frame
        real_Drone_orientation = pyq.Quaternion(self.drone_state_msg.actual_drone_orientation.w, -self.drone_state_msg.actual_drone_orientation.x, self.drone_state_msg.actual_drone_orientation.z, self.drone_state_msg.actual_drone_orientation.y)
        # turn into flattened rotation matrix, row major
        self._real_drone_rm[:] = [real_Drone_orientation.rotation_matrix[0][0], real_Drone_orientation.rotation_matrix[0][1], real_Drone_orientation.rotation_matrix[0][2], real_Drone_orientation.rotation_matrix[1][0], real_Drone_orientation.rotation_matrix[1][1], real_Drone_orientation.rotation_matrix[1][2], real_Drone_orientation.rotation_matrix[2][0], real_Drone_orientation.rotation_matrix[2][1], real_Drone_orientation.rotation_matrix[2][2]]
        if self.drone_state_msg.running:
            self._states[10] = True
            self._states[11] = False
        else:
            self._states[10] = False
            self._states[11] = True
        
    # handle the log state data
    def log_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.log_state_msg = msg
        self._states[0] = self.log_state_msg.opened
        self._states[1] = self.log_state_msg.closed
        
    # handle the rrim state data
    def rrim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.rrim_state_msg = msg
        if self.rrim_state_msg.running:
            self._states[8] = True
            self._states[9] = False
        else:
            self._states[8] = False
            self._states[9] = True
        
    # handle the sim state data
    def sim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        #self.get_logger().info('I heard position: "%f %f %f"' % (msg.position.x, msg.position.y, msg.position.z))
        self.sim_state_msg = msg
        self._sim_drone_pos[:] = [self.sim_state_msg.position.x, self.sim_state_msg.position.y, self.sim_state_msg.position.z]
        self._sim_drone_rm[:] = [self.sim_state_msg.rotation_matrix.m11, self.sim_state_msg.rotation_matrix.m12, self.sim_state_msg.rotation_matrix.m13, self.sim_state_msg.rotation_matrix.m21, self.sim_state_msg.rotation_matrix.m22, self.sim_state_msg.rotation_matrix.m23, self.sim_state_msg.rotation_matrix.m31, self.sim_state_msg.rotation_matrix.m32, self.sim_state_msg.rotation_matrix.m33]
        if self.sim_state_msg.running:
            self._states[6] = True
            self._states[7] = False
        else:
            self._states[6] = False
            self._states[7] = True
        
    # handle the brim state data
    def brim_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.brim_state_msg = msg
        if self.brim_state_msg.running:
            self._states[4] = True
            self._states[5] = False
        else:
            self._states[4] = False
            self._states[5] = True
        
    # handle the bmn state data
    def bmn_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg)
        self.bmn_state_msg = msg
        self._cmd_drone_pos = [self.bmn_state_msg.desired_drone_position.x, self.bmn_state_msg.desired_drone_position.y, self.bmn_state_msg.desired_drone_position.z]
        if self.bmn_state_msg.running:
            self._states[2] = True
            self._states[3] = False
        else:
            self._states[2] = False
            self._states[3] = True
        
    # publish gui controls commands
    def gui_controls_pub_callback(self):
        # publish the gui controls message
        msg = Guicontrols()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.open_logs = bool(self._controls[0])
        msg.close_logs = bool(self._controls[1])
        msg.clear_logs = bool(self._controls[2])
        msg.start_simulation = bool(self._controls[3])
        msg.stop_simulation = bool(self._controls[4])
        msg.reset_simulation = bool(self._controls[5])
        msg.start_bmn = bool(self._controls[6])
        msg.stop_bmn = bool(self._controls[7])
        msg.reset_bmn = bool(self._controls[8])
        msg.start_rpicomms = bool(self._controls[9])
        msg.stop_rpicomms = bool(self._controls[10])
        msg.reset_rpicomms = bool(self._controls[11])
        msg.takeoff = bool(self._controls[12])
        msg.land = bool(self._controls[13])
        self.gui_controls_pub.publish(msg)
        
class visualizations():
    def __init__(self, pid, eid, mt, rt, cls, sts, sp, sr, rp, rr, cp, up, sl, logger):
        # class vars
        self.part_id = pid
        self.exp_id = eid
        self.move_type = mt
        self.r_type = rt
        # logs open, logs close, logs clear, sim start, sim stop, sim reset, bmn start, bmn stop, bmn reset, rpicomms start, rpicomms stop, rpicomms reset, irl drone takeoff, irl drone land
        self.ctrls = cls
        # logs running, logs stopped, bmn running, bmn stopped, brim running, brim stopped, sim running, sim stopped, rrim running, rrim stopped, rpicomms running, rpicomms stopped, irl flying, irl landed
        self.sts = sts
        self.sim_pos = sp
        self.sim_rm = sr
        self.real_pos = rp
        self.real_rm = rr
        self.cmd_pos = cp
        self.updatetrue = up
        self.logsetuptrue = sl
        self.node_logger = logger
        
        root_folder = os.getcwd()
        # Load a mesh
        self.RoomV, self.RoomF = igl.read_triangle_mesh(os.path.join(root_folder, "Haptic-RW-UAS-Ground-Controller/src/visualization_inputs/resource", "Room_3m.obj"))
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
        
        # Initialize the polyscope window
        ps.set_autocenter_structures(True)
        ps.set_build_gui(False)
        ps.set_window_size(3500, 2200)
        ps.set_open_imgui_window_for_user_callback(True)

        ps.init()

        ps.look_at([-2.0, 1.5, -15.0], [0, 0, 0])

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
        
    def pub_updates(self):
        self.updatetrue.value = 1
    
    def pub_logs(self):
        self.logsetuptrue.value = 1
        
    def vis_callback(self):
        # Create a window called "Control Panel" and place the widgets below it
        ps.imgui.Begin("Control Panel", True)
        ps.imgui.PushItemWidth(100)
        ps.imgui.SetWindowFontScale(1.5)
        ps.imgui.SetWindowPos([10, 10])
        
        ps.imgui.Text("Command Position: %f %f %f" % (self.cmd_pos[0], self.cmd_pos[1], self.cmd_pos[2]))
        ps.imgui.Text("Simulated Position: %f %f %f" % (self.sim_pos[0], self.sim_pos[1], self.sim_pos[2]))
        ps.imgui.Text("Real Position: %f %f %f" % (self.real_pos[0], self.real_pos[1], self.real_pos[2]))
        
        # participant id iput box
        changed_pi, self.part_id.value = ps.imgui.InputInt("Participant Number", self.part_id.value)
        # experiment id input box
        changed_ei, self.exp_id.value = ps.imgui.InputInt("Experiment Number", self.exp_id.value)
        # test direction select menu
        changed_m, self.move_type.value= ps.imgui.Combo("Task Type", self.move_type.value, ["Guided Movement Task", "Contact Task"])
        # if self.mtnum == 0:
        #     self.move_type = "FM"
        # else:
        #     self.move_type = "GM"
            
        # rim type select menu
        changed_r, self.r_type.value = ps.imgui.Combo("CoSim Type", self.r_type.value, ["CoSim1", "CoSim2", "CoSim3", "RIM"])
        
        # logs controls, first thing to activate and last to close
        #self.node_logger.info("control commands sent open_logs: %d close_logs: %d clear_logs: %d start sim: %d stop sim: %d reset sim: %d start bmn: %d stop bmn: %d reset bmn: %d start rpicomms: %d stop rpicomms: %d reset rpicomms: %d takeoff: %d land: %d" % (self.controls[0], self.controls[1], self.controls[2], self.controls[3], self.controls[4], self.controls[5], self.controls[6], self.controls[7], self.controls[8], self.controls[9], self.controls[10], self.controls[11], self.controls[12], self.controls[13]))
        #self.node_logger.info("state values logs running: %d logs stopped: %d bmn running: %d bmn stopped: %d brim running: %d brim stopped: %d sim running: %d sim stopped: %d rrim running: %d rrim stopped: %d rpicomms running: %d rpicomms stopped: %d irl flying: %d irl landed: %d" % (self._states[0], self.states[1], self.states[2], self.states[3], self.states[4], self.states[5], self.states[6], self.states[7], self.states[8], self.states[9], self.states[10], self.states[11], self.states[12], self.states[13]))
        # name logs button
        # if button pressed and logs are currently closed
        if (ps.imgui.Button("Name Logs") and self.sts[1]):
            self.node_logger.info('Naming Logs')
            self.pub_logs()
            # publish the log setup message
            #msg = Logsetup()
            #msg.header.stamp = self.get_clock().now().to_msg()
            #msg.participant_id = self.part_id
            #msg.experiment_id = self.exp_id
            #msg.movement_type = self.move_type
            #msg.rim_type = self.r_type
            #self.log_setup_pub.publish(msg)
            
        # open logs button
        # if button pressed and logs are currently closed
        if (ps.imgui.Button("Open Logs") and self.sts[1]):
            self.node_logger.info('Opening Logs')
            self.ctrls[0] = True
            self.ctrls[1] = False
            self.ctrls[2] = False
            
        # close logs button
        # if button pressed and logs are currently open
        if (ps.imgui.Button("Close Logs") and self.sts[0]):
            self.node_logger.info('Closing Logs')
            self.ctrls[0] = False
            self.ctrls[1] = True
            self.ctrls[2] = False        
        
        # sim ctrls, first after logs to activate, last before logs to close
        # start simulation button
        # if button pressed and sim is currently closed and logs are open already
        # self.sts = [False, True, False, True, False, True, False, True, False, True, False, True, False, False]
        #if (ps.imgui.Button("Start Simulation") and self.sts[7] and self.sts[0]):
        if (ps.imgui.Button("Start Simulation")):
            self.node_logger.info('Starting Simulation')
            self.ctrls[3] = True
            self.ctrls[4] = False
            self.ctrls[5] = False
        
        # stop simulation button
        # if button pressed and sim is currently open and bmn/brim is closed and rpicomms/rrim is closed
        #if (ps.imgui.Button("Stop Simulation") and self.sts[6] and self.sts[5] and self.sts[3] and self.sts[11] and self.sts[9]):
        if (ps.imgui.Button("Stop Simulation")):
            self.node_logger.info('Stopping Simulation')
            self.ctrls[3] = False
            self.ctrls[4] = True
            self.ctrls[5] = False            
            
        # bmn and brim ctrls, have to be activated after sim, stopped before sim, open bmn first and close brim first
        # start bmn button
        # if button pressed and bmn/brim is closed and sim is started already
        #if (ps.imgui.Button("Start BMN") and self.sts[5] and self.sts[3] and self.sts[6]):
        if (ps.imgui.Button("Start BMN")):
            self.node_logger.info('Starting BMN')
            self.ctrls[6] = True
            self.ctrls[7] = False
            self.ctrls[8] = False
            
        # stop bmn button
        # if button pressed and bmn/brim is open
        #if (ps.imgui.Button("Stop BMN") and self.sts[4] and self.sts[2]):
        if (ps.imgui.Button("Stop BMN")):
            self.node_logger.info('Stopping BMN')
            self.ctrls[6] = False
            self.ctrls[7] = True
            self.ctrls[8] = False
            
        # rpicomms and rrim ctrls, have to activate after sim, stopped before sim, open rpicomms first and close rrim first
        # start rpicomms button
        # if button pressed and rpicomms/rrim is closed and sim is started already
        #if (ps.imgui.Button("Start Rpicomms") and self.sts[11] and self.sts[9] and self.sts[6]):
        if (ps.imgui.Button("Start Rpicomms")):
            self.node_logger.info('Starting Rpicomms')
            self.ctrls[9] = True
            self.ctrls[10] = False
            self.ctrls[11] = False
            
        # stop rpicomms button
        # if button pressed and rpicomms/rrim is open
        #if (ps.imgui.Button("Stop Rpicomms") and self.sts[10] and self.sts[8]):
        if (ps.imgui.Button("Stop Rpicomms")):
            self.node_logger.info('Stopping Rpicomms')
            self.ctrls[9] = False
            self.ctrls[10] = True
            self.ctrls[11] = False        
        
        ps.imgui.PopItemWidth()
        ps.imgui.End()
        
        self.pub_updates()
        
        # update the drone objects position and orientation using the info from the ros2 msgs
        drows = self.DroneVOG.shape[0]
        d = np.ones((drows))
        DroneVnew = np.zeros((drows, 3))
        DroneVnew[:, 0] = -self.sim_pos[0] * d
        DroneVnew[:, 1] = (self.sim_pos[2]-1.25) * d
        DroneVnew[:, 2] = self.sim_pos[1] * d
        #self.node_logger.info("sim drone position: %f %f %f" % (-self.sim_pos[0], self.sim_pos[2]-5, self.sim_pos[1]))
        sim_rm = np.array(self.sim_rm[:]).reshape(3, 3)
        self.DroneV = np.transpose(np.dot(sim_rm, np.transpose(self.DroneVOG))) + (1000 * DroneVnew)
        ps.get_surface_mesh("Drone").update_vertex_positions(self.DroneV)
        
        rdrows = self.RDroneVOG.shape[0]
        rd = np.ones((rdrows))
        RDroneVnew = np.zeros((rdrows, 3))
        RDroneVnew[:, 0] = -self.real_pos[0] * rd
        RDroneVnew[:, 1] = (self.real_pos[2]-1.25) * rd
        RDroneVnew[:, 2] = self.real_pos[1] * rd
        # hopefully reshaping the rotation matrix is consistent between the two drones
        real_rm = np.array(self.real_rm[:]).reshape(3, 3)
        self.RDroneV = np.transpose(np.dot(real_rm, np.transpose(self.RDroneVOG))) + (1000 * RDroneVnew)
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