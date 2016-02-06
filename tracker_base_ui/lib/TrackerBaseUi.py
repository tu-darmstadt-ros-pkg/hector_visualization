import os
import rospy
import rospkg
import math

from PyQt4 import QtGui, QtCore
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QModelIndex, QObject, Qt, QTimer, Signal, Slot, QTimer
from python_qt_binding.QtGui import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView, QPen, QBrush, QColor, QTransform
from rospy import Time
from std_msgs.msg import Bool
from std_msgs.msg import Float64, Float32, Empty


class TrackerBaseUi(QObject):
    _update_task_delegates = Signal()
    
    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('TrackerBaseUi')

        # setup publisher
        self._WhiteLightPublisher = rospy.Publisher('/light', Bool)
        self._BlueLightPublisher = rospy.Publisher('/bluelight', Bool)
        self._CalibFlipperPublisher = rospy.Publisher('/init_flipper', Empty)
        self._FlipperCommandPublisher = rospy.Publisher('/flipper_control/command/absolute', Float64)

	self._FlipperStateSubsciber = rospy.Subscriber('/flipper_control/state', Float64, self._on_flipper_state)

        # setup main widget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('tracker_base_ui'), 'lib', 'TrackerBaseUi.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('TrackerBaseUi')

        #set connections
        self._widget.white_checkBox.stateChanged.connect(self._on_white_changed)
        self._widget.blue_checkBox.stateChanged.connect(self._on_blue_changed)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # setup subscribers  
        self._trackerSupplySubscriber = rospy.Subscriber("/supply_voltage", Float32, self._on_tracker_supply)
        

        # connect Signal Slot
        self._update_task_delegates.connect(self._on_update_task_delegates)
        self._widget.calib_flipper_button.pressed.connect(self._on_calib_flipper_button_pressed)
	self._widget.flipper_up_button.pressed.connect(self._flipper_up_button_pressed) 
	self._widget.flipper_center_button.pressed.connect(self._flipper_center_button_pressed)
	self._widget.flipper_down_button.pressed.connect(self._flipper_down_button_pressed)
        self.supply = -1
        self._widget.supply_lineEdit.setText('unknown')
        # init percept model

	self.flipper_scene = QtGui.QGraphicsScene(self)

        self._widget.graphicsView_Flipper.setScene(self.flipper_scene)
	self.flipper_state = 0;
	self._widget.graphicsView_Flipper.setRenderHint(QtGui.QPainter.Antialiasing)

	self._widget.graphicsView_Flipper.show()

 	self.timer = QTimer()
	self.timer.timeout.connect(self.draw_flipper_state)
	self.timer.start(500)

	self.flipper_front_max_state = 0.79
	self.flipper_front_min_state = -0.2

	self.flipper_diff = 0.1
        
    def _on_tracker_supply(self, message):
        self.supply = message.data
        self._update_task_delegates.emit()

    def _on_flipper_state(self, message):
	self.flipper_state = message.data

    def _on_flipper_cmd(self, message):
	self.flipper_cmd +=0.1 #TODO

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
   
    def _on_white_changed(self,value):
        self._WhiteLightPublisher.publish(value)

    def _on_blue_changed(self,value):
        self._BlueLightPublisher.publish(value)

    def _on_calib_flipper_button_pressed(self):
        self._CalibFlipperPublisher.publish(Empty())

    def _flipper_up_button_pressed(self):
	self._FlipperCommandPublisher.publish(0.5)
	self.flipper_state += 0.05

    def _flipper_center_button_pressed(self):
	self._FlipperCommandPublisher.publish(0)
	self.flipper_state = 0

    def _flipper_down_button_pressed(self):
	self._FlipperCommandPublisher.publish(-0.2)    
	self.flipper_state -= 0.05

    def _on_update_task_delegates(self):
        if (self.supply == -1) :
            self._widget.supply_lineEdit.setText('unknown')
        else:
            self._widget.supply_lineEdit.setText(str(self.supply))

    def compute_coordinates(self, angle):

	length = 100
	y = length * math.cos(-angle)
	x = length * math.sin(-angle)
        return {'y':y, 'x':x }

    def draw_flipper_max(self):
	coords = self.compute_coordinates(self.flipper_front_max_state)

	self.flipper_scene.addLine(200, 50+coords['x'], 300, 50+coords['x'])

    def draw_flipper_min(self):
	coords = self.compute_coordinates(self.flipper_front_min_state)

	self.flipper_scene.addLine(200, 100+coords['x'], 300, 100+coords['x'])

    def draw_flipper_state(self):
	length = 100

	self._widget.flipper_front_current.setText(str(self.flipper_state)) 

	#self.flipper_state += self.flipper_diff;

	if(self.flipper_state > self.flipper_front_max_state):
		self.flipper_diff *= -1	
	#	self.flipper_state = self.flipper_front_max_state

	if(self.flipper_state < self.flipper_front_min_state):
		self.flipper_diff *= -1	
	#	self.flipper_state = self.flipper_front_min_state

	self.flipper_scene.clear()

	self.draw_flipper_max()
	self.draw_flipper_min()

	#red chassi
	self.flipper_scene.addRect(65, 60, 145, 25, QPen(), QBrush(QColor(255,0,0)))

	#right wheel
	self.flipper_scene.addEllipse(150, 50, 50, 50, QPen(), QBrush(QColor(0,0,0)))

	#left wheel
	self.flipper_scene.addEllipse(50, 50, 50, 50, QPen(), QBrush(QColor(0,0,0)))
	
	#flipper wheel
	transform = QTransform()
	transform.translate(155,70)
	transform.rotateRadians(-self.flipper_state)
	transform.translate(-25,-25)

	#flipper wheel connection
	flipper_wheel = QtGui.QGraphicsEllipseItem(120,5, 50, 50)
	flipper_wheel.setTransform(transform)
	flipper_wheel.setBrush( QBrush(QColor(0,0,0)))

	self.flipper_scene.addItem(flipper_wheel)
	#self.flipper_scene.addEllipse(250, 50, 50, 50, QPen(), QBrush(QColor(0,0,0)))


	transform = QTransform()
	transform.translate(160,75)
	transform.rotateRadians(-self.flipper_state)
	transform.translate(-10,-10)
	flipper_link = QtGui.QGraphicsRectItem(0,0, 135, 20)
	flipper_link.setBrush( QBrush(QColor(163,163,163)))

	flipper_link.setTransform(transform)

	self.flipper_scene.addItem(flipper_link)

	#front connection
	self.flipper_scene.addEllipse(155, 70, 10, 10, QPen(), QBrush(QColor(0,0,0)))

	transform = QTransform()
	transform.translate(160,75)
	transform.rotateRadians(-self.flipper_state)
	transform.translate(-10,-10)

	#flipper wheel connection
	flipper_wheel_connection = QtGui.QGraphicsEllipseItem(120,5, 10, 10)
	flipper_wheel_connection.setTransform(transform)
	flipper_wheel_connection.setBrush( QBrush(QColor(0,0,0)))

	self.flipper_scene.addItem(flipper_wheel_connection)

#	self.flipper_scene.addEllipse(270, 70, 10, 10, QPen(), QBrush(QColor(255,255,255)))

