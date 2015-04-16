import roslib

roslib.load_manifest('hector_rqt_joint_control')
from time import strftime, localtime
import rospy
import sys
import copy

import time
import math

from python_qt_binding.QtCore import Signal, Slot, Qt, QObject
from python_qt_binding.QtGui import QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QFrame, QSlider, \
    QProgressBar, QSpacerItem, QRadioButton, QButtonGroup, QCheckBox  # QDoubleSpinBox, QFileDialog

from urdf_parser_py.urdf import URDF

#from atlas_msgs.msg import AtlasCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#import moveit_commander


class Joint(object):
    def __init__(self, name, controller):
        self.name = name
        self.position = 0.0
        self.current_position = 0.0
        self.oldPose = 0.0
        self.velocity = 0.0
        self.effort = 0.0
        self.controller = controller
        self.slicheccheder = None
        self.progress_bar = None

    def on_slider_moved(self, value):
        new_value = value / 10000.0
        self.position = copy.deepcopy(new_value)

        self.controller.value_changed = True
	self.controller.publish_joint_state_command()

class Controller(object):
    def __init__(self, parent, label, name, topic):
        self.label = label
        self.name = name
        self.parent = parent
        self.joints = []
        self.duration = 0.25
        self.min_duration = 0.25
        self.max_vel = 0.785  # 45 deg/s
        self.max_cmd_change = 0.785  # 45 deg/s

        self.oldPoses = False
        self.override = False
        self.saveFile = ""
        # self.ghost_joints = []
        self.pub_robot = rospy.Publisher('/joint_controllers/' + self.name + '/command',
                                         JointTrajectory, queue_size=10)
        self.pub_ghost = rospy.Publisher('/flor/ghost/set_joint_states', JointState, queue_size=10)

        self.joint_state_pub = rospy.Publisher(topic, JointState, queue_size=10)
 
    ##
    # Create for each Action its own callback so that the controller and the action name can be used
    ##
    def create_action_callback(self, action_name):
        def process_action():
        	rospy.logerr("Pressed:" + action_name+" in controller "+self.name)
		for joint in self.joints:
			joint_target_position = rospy.get_param("/hector_rqt_joint_control/" + action_name + "/joint_positions/"+joint.name, joint.position)
			rospy.logerr("/hector_rqt_joint_control/" + action_name + "/joint_positions/"+joint.name+": "+ str(joint_target_position))
			rospy.logerr("Moving joint " + joint.name+" from "+str(joint.position)+" to "+ str(joint_target_position))			
			joint.position = joint_target_position
                	joint.slider.setValue(int(joint_target_position * 10000.0))
	    
        return process_action

    ##
    # Publish the JointState as command
    ##
    def publish_joint_state_command(self):
	print "publishing new joint states for " + self.name
	joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for joint in self.joints:
            joint_state.name.append(joint.name)
            joint_state.position.append(joint.position)
            #joint_state.velocity.append(joint.velocity)
            #joint_state.effort.append(joint.effort)

        # print joint_state
        self.joint_state_pub.publish(joint_state)

    def publish_robot_joint_state(self):
        print "publishing new joint states for " + self.name + " to Atlas"
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = len(self.joints) * ["1"]
        joint_trajectory.points = [JointTrajectoryPoint(), JointTrajectoryPoint()]
        joint_trajectory.points[0].positions = len(self.joints) * [0.0]
        joint_trajectory.points[1].positions = len(self.joints) * [0.0]
        # joint_trajectory.points[0].velocities = len(self.joints)*[0.0]
        # joint_trajectory.points[1].velocities = len(self.joints)*[0.0]
        joint_trajectory.points[0].time_from_start = rospy.Duration(0.000)
        joint_trajectory.points[1].time_from_start = rospy.Duration(self.duration)

        any_name_not_found = False
        ok_to_publish = True
        self.duration = self.min_duration  # initialize duration

        for i, joint in enumerate(self.joints):
            not_found_name = True
            # if joint is there
            for j in range(len(self.parent.parent.joint_states.name)):
                if joint.name == self.parent.parent.joint_states.name[j]:
                    not_found_name = False
                    joint_trajectory.joint_names[i] = copy.deepcopy(joint.name)
                    joint_trajectory.points[0].positions[i] = copy.deepcopy(self.parent.joint_commands[j])

            if not_found_name:
                print "Name not found=", joint.name, " in controller ", self.label
                any_name_not_found = True

            joint_trajectory.points[1].positions[i] = copy.deepcopy(joint.position)

            diff_value = math.fabs(joint_trajectory.points[1].positions[i] - joint_trajectory.points[0].positions[i])
            if diff_value > self.max_cmd_change:
                ok_to_publish = False
                print "Requested change is too large = ", diff_value, " for joint ", joint.name
            else:
                time_value = diff_value / self.max_vel
                if time_value > self.duration:
                    self.duration = copy.deepcopy(time_value)
                    print "Clamp velocity to ", self.max_vel, " from ", diff_value, "/", self.min_duration, " =", \
                        diff_value / self.min_duration, " with duration=", self.duration

        if any_name_not_found:
            print "Failed to find a joint name - ignore command"
            print self.parent.parent.joint_states
        else:
            if ok_to_publish | self.override:
                joint_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.25)
                print joint_trajectory
                self.pub_robot.publish(joint_trajectory)
            else:
                print "Cannot to publish due to change error"

    def publish_ghost_joint_state(self):
        # print "publishing new joint states for "+self.name+" to the ghost robot"
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        for joint in self.joints:
            joint_state.name.append(joint.name)
            joint_state.position.append(joint.position)
            joint_state.velocity.append(joint.velocity)
            joint_state.effort.append(joint.effort)

        # print joint_state
        self.pub_ghost.publish(joint_state)

    def on_snap_ghost_pressed(self):
        print "Snap ", self.name, " values to current ghost joint positions"
        self.block_signals(True)
        self.reset_ghost_joint_sliders()
        self.block_signals(False)

    def on_snap_current_pressed(self):
        print "Snap ", self.name, " values to current joint positions"
        self.block_signals(True)
        self.reset_current_joint_sliders()
        self.block_signals(False)

    def on_apply_robot_pressed(self):
        print "Send latest joint values directly to robot", self.name
        self.oldPoses = True
        for i, joint in enumerate(self.joints):
            joint.oldPose = joint.position
        self.publish_robot_joint_state()

    def on_save_joints_pressed(self):
        print "Saving latest joint values to file"
        if self.saveFile == "":
            self.create_save_file()
        print "Appending save file"
        save_file = open(self.saveFile, 'a')
        write_buff = strftime('%H:%M:%S', localtime()) + ',    '
        for i, joint in enumerate(self.joints):
            write_buff += str(joint.position) + ',  '
        save_file.write(write_buff[0:(len(write_buff) - 3)] + '\n')
        save_file.close()

    def on_undo_pressed(self):
        print "Undo requested on ", self.name
        if self.oldPoses:
            for i, joint in enumerate(self.joints):
                joint.position = joint.oldPose
                joint.slider.setValue(int(joint.oldPose * 10000.0))
        else:
            print "No old action to undo!"

    def set_override(self, ovr):
        self.override = ovr

    def create_save_file(self):
        print "No old file found creating new save file."
        self.saveFile = (
            roslib.packages.get_pkg_dir('vigir_rqt_position_mode') + '/launch/' + self.name + '_' + strftime(
                '%m-%d_%H:%M:%S', localtime()) + '.txt')
        print "File location  = ", self.saveFile
        save_file = open(self.saveFile, 'w')
        # self.saveFile = temp.name;
        line2 = 'indicies, '
        line1 = self.name + ', '
        for i, joint in enumerate(self.joints):
            line1 += ',' + joint.name
            if joint.name == 'back_bkz':
                line2 += '1, '
            if joint.name == 'back_bky':
                line2 += '2, '
            elif joint.name == 'back_bkx':
                line2 += '3, '
            elif joint.name == 'l_leg_hpz':
                line2 += '4, '
            elif joint.name == 'l_leg_hpx':
                line2 += '5, '
            elif joint.name == 'l_leg_hpy':
                line2 += '6, '
            elif joint.name == 'l_leg_kny':
                line2 += '7, '
            elif joint.name == 'l_leg_aky':
                line2 += '8, '
            elif joint.name == 'l_leg_akx':
                line2 += '9, '
            elif joint.name == 'r_leg_hpz':
                line2 += '10, '
            elif joint.name == 'r_leg_hpx':
                line2 += '11, '
            elif joint.name == 'r_leg_hpy':
                line2 += '12, '
            elif joint.name == 'r_leg_kny':
                line2 += '13, '
            elif joint.name == 'r_leg_aky':
                line2 += '14, '
            elif joint.name == 'r_leg_akx':
                line2 += '15, '
            elif joint.name == 'l_arm_shz':
                line2 += '16, '
            elif joint.name == 'l_arm_shx':
                line2 += '17, '
            elif joint.name == 'l_arm_ely':
                line2 += '18, '
            elif joint.name == 'l_arm_elx':
                line2 += '19, '
            elif joint.name == 'l_arm_wry':
                line2 += '20, '
            elif joint.name == 'l_arm_wrx':
                line2 += '21, '
            elif joint.name == 'r_arm_shz':
                line2 += '22, '
            elif joint.name == 'r_arm_shx':
                line2 += '23, '
            elif joint.name == 'r_arm_ely':
                line2 += '24, '
            elif joint.name == 'r_arm_elx':
                line2 += '25, '
            elif joint.name == 'r_arm_wry':
                line2 += '26, '
            elif joint.name == 'r_arm_wrx':
                line2 += ',27 '
        save_file.write(line1[0:7] + line1[8:] + '\n')
        save_file.write(line2[0:len(line2) - 2] + '\n')
        save_file.close()

    def block_signals(self, block):
        for joint in self.joints:
            joint.slider.blockSignals(block)

    def reset_current_joint_sliders(self):
        for joint in self.joints:
            print "Setting joint ", joint.name, " = ", str(joint.current_position)
            joint.slider.setValue(int(joint.current_position * 10000.0))
            joint.position = joint.current_position

    def reset_ghost_joint_sliders(self):
        for i, joint in enumerate(self.joints):
            for j, ghost_joint_name in enumerate(self.parent.parent.ghost_joint_states.name):
                # if joint is there
                if ghost_joint_name == joint.name:
                    print "Setting ghost joint ", ghost_joint_name, " = ", str(
                        self.parent.parent.ghost_joint_states.position[j])
                    joint.slider.setValue(int(self.parent.parent.ghost_joint_states.position[j] * 10000.0))
                    joint.position = self.parent.parent.ghost_joint_states.position[j]

    def __del__(self):
        print "Shutdown controller ", self.name
        self.pub_robot.unregister()
        self.pub_ghost.unregister()


class JointControl(object):
    def __init__(self, parent, file_name, widget):
	
	print "JointControl __init__ --start"
        self.controllers = []
        self.joint_commands = []
        self.parent = parent

        # self.loadFile(file_name)
        self.load_robot_model()


        #print "--> Robot Joints: <--"
        #print len(robot.joints)
        #print robot.joints[0]

        for controller in self.controllers:
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel)
            frame.setFrameShadow(QFrame.Raised)

            vbox = QVBoxLayout()
            label = QLabel()
            label.setText(controller.label)
            vbox.addWidget(label)
            print controller.name

	    group_actions = rospy.get_param("/hector_rqt_joint_control/" + controller.group_name + "/actions", ())

	    for action in group_actions:
		action_label = rospy.get_param("/hector_rqt_joint_control/" + action + "/label", action)
		rospy.logerr("processing action: %s", action)
		controller.snap_to_ghost_button = QPushButton(action_label)
            	controller.snap_to_ghost_button.pressed.connect(controller.create_action_callback(action))
            	vbox.addWidget(controller.snap_to_ghost_button)

	    rospy.logerr("group_actions: %s", group_actions)

            for joint in controller.joints:
                label = QLabel()
                label.setText(joint.name)
                vbox.addWidget(label);

		rospy.logerr("Processing joint: %s", joint)

                robot_joint = self.robot.joints[self.joint_list[joint.name]]
                
		if hasattr(robot_joint, 'limit') & hasattr(robot_joint.limit, 'lower') & hasattr(robot_joint.limit, 'upper'):
			lower_limit = robot_joint.limit.lower
			upper_limit = robot_joint.limit.upper
		else :	
			rospy.logwarn("Found link without limits. What to do??")			
			lower_limit = 0
			upper_limit = 1

                joint.slider = QSlider(Qt.Horizontal)

                joint.slider.setRange(int(lower_limit * 10000.0), int(upper_limit* 10000.0))
                joint.slider.setValue(int(lower_limit* 10000.0))
                joint.slider.setSingleStep((upper_limit - lower_limit) / 20.0)
                joint.slider.valueChanged.connect(joint.on_slider_moved)
                vbox.addWidget(joint.slider)
                joint.progress_bar = QProgressBar()
                joint.progress_bar.setRange(int(lower_limit* 10000.0),
                                            int(upper_limit* 10000.0))
                joint.progress_bar.setValue(int(lower_limit* 10000.0))
                vbox.addWidget(joint.progress_bar)

            vbox.addStretch()

            frame.setLayout(vbox)
            widget.addWidget(frame)

    ##
    # Load Robot Model from parameter server and create all groups/controller with their joints with respect to the config file
    ## 
    def load_robot_model(self):

        self.robot = URDF.from_parameter_server()

	self.joint_list = {}
        for ndx, jnt in enumerate(self.robot.joints):
            self.joint_list[jnt.name] = ndx
	    rospy.logdebug("Joint %d: %s",ndx, jnt.name)


        selected_group_names = rospy.get_param("/hector_rqt_joint_control/groups", ())

	rospy.logdebug("Selected Group Names: %s", selected_group_names)

	#create joint groups
        for group_name in selected_group_names:
	    rospy.logerr( "Processing group: %s",group_name)
            controller_name = rospy.get_param("/hector_rqt_joint_control/" + group_name + "/controller_name", group_name)
            controller_label = rospy.get_param("/hector_rqt_joint_control/" + group_name + "/label", group_name)
            controller_topic = rospy.get_param("/hector_rqt_joint_control/" + group_name + "/topic", "/"+group_name+"_joint_cmd")
            controller_joints = rospy.get_param("/hector_rqt_joint_control/" + group_name + "/joints", ())
	    controller = Controller(self, controller_label, controller_name, controller_topic)
	    controller.group_name = group_name
	    intersect_joints = (joint for joint in controller_joints if joint in self.joint_list )
            for joint_name in controller_joints:
                controller.joints.append(Joint(joint_name, controller))
	    self.controllers.append(controller)


   
    def load_file(self, file_name):
        if file_name[0]:
            print "Loading ", file_name
            load_file_ptr = open(file_name, 'r')
            while True:
                line = load_file_ptr.readline();
                if line != "":
                    line = line.strip()
                    split_line = line.split(',')
                    if split_line[0][0] != '#':
                        try:
                            param_joint_names = rospy.get_param(split_line[2].strip())
                        except KeyError:
                            print 'Key', split_line[2].strip(), 'not found on parameter server.'
                        else:
                            self.controllers.append(Controller(self, split_line[0].strip(), split_line[1].strip()))
                            for joint_name in param_joint_names:
                                self.controllers[-1].joints.append(Joint(joint_name, self.controllers[-1]))
                                # self.controllers[-1].ghost_joints.append(Joint(joint_name,self.controllers[-1]))
                else:
                    break
        else:
            print "Load cancelled."
            sys.exit(-1)

    def update_joint_positions(self, joint_states, first_time):
        if first_time:
            # initialize wbc command vector
            print "Initialize joint commands on first time"
            self.joint_commands = joint_states.position

        # need to find the joint in our structures
        for controller in self.controllers:
            for joint in controller.joints:
                not_found_name = True
                # if joint is there
                for i in range(len(joint_states.name)):
                    if joint.name == joint_states.name[i]:
                        not_found_name = False
                        joint.current_position = joint_states.position[i]
                        # joint.velocity         = joint_states.velocity[i]
                        # joint.effort           = joint_states.effort[i]
                        joint.progress_bar.setValue(int(joint.current_position * 10000.0))
                        joint.progress_bar.setFormat("%.2f" % joint.current_position)  # joint.position)
                if not_found_name:
                    print "Name not found=", joint.name, " in controller ", controller.label
                    print joint_states

    def block_signals(self, block):
        for controller in self.controllers:
            controller.block_signals(block)

    def reset_current_joint_sliders(self):
        for controller in self.controllers:
            controller.block_signals(True)
            controller.reset_current_joint_sliders()
            controller.block_signals(False)

    def reset_ghost_joint_sliders(self):
        for controller in self.controllers:
            controller.block_signals(True)
            controller.reset_ghost_joint_sliders()
            controller.block_signals(False)

class JointControlWidget(QObject):
    updateStateSignal = Signal(object)
    updateGhostSignal = Signal(object)
    updateCommandSignal = Signal(object)

    def __init__(self, context):
        super(JointControlWidget, self).__init__()
        self.updateStateSignal.connect(self.on_update_state)
        self.updateGhostSignal.connect(self.on_update_ghost)
        self.updateCommandSignal.connect(self.on_update_command)

        self.joint_states = JointState()
        self.ghost_joint_states = JointState()
        self._widget = context
        vbox = QVBoxLayout()

        # Define checkboxes
        radios = QWidget()
        hbox_radio = QHBoxLayout()
        self.radioGroup = QButtonGroup()
        self.radioGroup.setExclusive(True)
        self.radio_ghost_target = QRadioButton()
        self.radio_ghost_target.setText("Ghost")
        self.radioGroup.addButton(self.radio_ghost_target, 0)
        self.radio_ghost_target.setChecked(True)
        self.radio_robot_target = QRadioButton()
        self.radio_robot_target.setText("Robot")
        self.radioGroup.addButton(self.radio_robot_target, 1)
        #hbox_radio.addStretch()
        #hbox_radio.addWidget(self.radio_ghost_target)
        # hbox_radio.addWidget(QLabel("Ghost"))
        #hbox_radio.addStretch()
        #hbox_radio.addWidget(self.radio_robot_target)
        # hbox_radio.addWidget(QLabel("Robot"))
        #hbox_radio.addStretch()
        #radios.setLayout(hbox_radio)

        #vbox.addWidget(radios)

        widget = QWidget()
        hbox = QHBoxLayout()

        # Left to right layout
        self.joint_control = JointControl(self,
                                          roslib.packages.get_pkg_dir('hector_rqt_joint_control') + '/launch/joints.txt',
                                          hbox)

        widget.setLayout(hbox)

        vbox.addWidget(widget)

        print "Add buttons to apply all ..."
        all_widget = QWidget()
        all_box = QHBoxLayout()

        self.snap_to_ghost_button = QPushButton("SnapAllGhost")
        self.snap_to_ghost_button.pressed.connect(self.on_snap_ghost_pressed)
        all_box.addWidget(self.snap_to_ghost_button)
        self.snap_to_current_button = QPushButton("SnapAllCurrent")
        self.snap_to_current_button.pressed.connect(self.on_snap_current_pressed)
        all_box.addWidget(self.snap_to_current_button)
        self.apply_to_robot_button = QPushButton("ApplyAllRobot")
        self.apply_to_robot_button.pressed.connect(self.on_apply_robot_pressed)
        all_box.addWidget(self.apply_to_robot_button)
        self.apply_to_robot_button = QPushButton("Apply WBC Robot")
        self.apply_to_robot_button.pressed.connect(self.on_apply_wbc_robot_pressed)
        all_box.addWidget(self.apply_to_robot_button)

        all_widget.setLayout(all_box)
        #vbox.addWidget(all_widget)

        override_box = QHBoxLayout()

        self.override = QCheckBox()
        self.override.setChecked(False)
        self.override.stateChanged.connect(self.on_override_changed)
        override_box.addWidget(self.override)

        override_label = QLabel("SAFETY OVERRIDE")
        override_label.setStyleSheet('QLabel { color: red }')

        override_box.addWidget(override_label)

        override_box.addStretch()

        #vbox.addLayout(override_box)

        #vbox.addStretch()

        self._widget.setLayout(vbox)

        self.first_time = True

        #self.commandSubscriber = rospy.Subscriber('/atlas/atlas_command', AtlasCommand, self.command_callback_fnc)

        self.ghostSubscriber = rospy.Subscriber('/flor/ghost/get_joint_states', JointState, self.ghost_callback_fnc)
        self.wbc_robot_pub = rospy.Publisher('/flor/wbc_controller/joint_states', JointState, queue_size=10)

	jointstate_topics = rospy.get_param("/hector_rqt_joint_control/state_topics", ())
	
	self.stateSubscriberList = [];
	for topic in jointstate_topics:
		self.stateSubscriberList.append( rospy.Subscriber(topic, JointState, self.state_callback_fnc))
        
        self.time_last_update_state = time.time()
        self.time_last_update_ghost = time.time()

    def shutdown_plugin(self):
        # Just make sure to stop timers and publishers, unsubscribe from Topics etc in the shutdown_plugin method.
        print "Shutdown ..."
        #self.stateSubscriber.unregister()
	
	for subscriber in self.stateSubscriberList:
		subscriber.unregister()

        self.ghostSubscriber.unregister()
        self.wbc_robot_pub.unregister()
        print "Done!"

    def state_callback_fnc(self, joint_state_msg):
        self.updateStateSignal.emit(joint_state_msg)

    def command_callback_fnc(self, atlas_command_msg):
        self.updateCommandSignal.emit(atlas_command_msg)

    def ghost_callback_fnc(self, ghost_joint_state_msg):
        self.updateGhostSignal.emit(ghost_joint_state_msg)

    def on_override_changed(self):
        if self.override.isChecked():
            print "WARNING: TURNING OFF SAFETY"
            for controller in self.joint_control.controllers:
                controller.set_override(True)
        else:
            print "Enabling safety checks"
            for controller in self.joint_control.controllers:
                controller.set_override(False)

    def on_snap_ghost_pressed(self):
        print "Snap all joint values to current ghost joint positions"
        for controller in self.joint_control.controllers:
            controller.on_snap_ghost_pressed()

    def on_snap_current_pressed(self):
        print "Snap all joint values to current joint positions"
        for controller in self.joint_control.controllers:
            controller.on_snap_current_pressed()

    def on_apply_robot_pressed(self):
        print "Send all latest joint values directly to robot"
        for controller in self.joint_control.controllers:
            controller.on_apply_robot_pressed()

    def on_apply_wbc_robot_pressed(self):
        print "Send all latest joint values directly to robot as a WBC command"
        if self.first_time:
            print "Uninitialized !"
        else:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = copy.deepcopy(self.joint_states.name)
            joint_state.position = list(copy.deepcopy(self.joint_states.position))
            joint_state.velocity = list(copy.deepcopy(self.joint_states.velocity))

            # print "Positions: ",joint_state.position
            for i, name in enumerate(joint_state.name):
                # for each joint, retrieve the data from each controller to populate the WBC vectors
                for controller in self.joint_control.controllers:
                    for joint in controller.joints:
                        if joint.name == self.joint_states.name[i]:
                            print "     fetching value for " + name + " = " + str(joint.position) + "  at ndx=" + str(i)
                            joint_state.position[i] = joint.position
                            joint_state.velocity[i] = joint.velocity
                            # joint_state.effort[i]   = joint.effort

            self.wbc_robot_pub.publish(joint_state)

    # this runs in the Qt GUI thread and can therefore update the GUI
    def on_update_command(self, atlas_command_msg):
        self.joint_control.joint_commands = atlas_command_msg.position

    def on_update_state(self, joint_state_msg):
        if time.time() - self.time_last_update_state >= 0.2:
            self.joint_states = joint_state_msg
            self.joint_control.update_joint_positions(self.joint_states, self.first_time)
            if self.first_time:
                self.joint_control.reset_current_joint_sliders()
                self.first_time = False
                #self.ghost_joint_states = copy.deepcopy(atlas_state_msg)
                #self.ghost_joint_states.position = list(self.ghost_joint_states.position)
            self.time_last_update_state = time.time()

    # this runs in the Qt GUI thread and can therefore update the GUI
    def on_update_ghost(self, ghost_joint_state_msg):
        if time.time() - self.time_last_update_ghost >= 0.2:
            for i, new_joint_name in enumerate(list(ghost_joint_state_msg.name)):
                name_found = False
                for j, stored_joint_name in enumerate(self.ghost_joint_states.name):
                    if new_joint_name == stored_joint_name:
                        name_found = True
                        self.ghost_joint_states.position[j] = ghost_joint_state_msg.position[i]
                if not name_found:
                    # Update the list without regard to order
                    self.ghost_joint_states.name.append(new_joint_name)
                    self.ghost_joint_states.position.append(ghost_joint_state_msg.position[i])
            self.time_last_update_ghost = time.time()


