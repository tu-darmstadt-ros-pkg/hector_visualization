import os
import rospy
import rospkg
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QModelIndex, QObject, Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView
from rospy import Time
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class TrackerBaseUi(QObject):
    _update_task_delegates = Signal()
    
    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('TrackerBaseUi')

        # setup publisher
        self._WhiteLightPublisher = rospy.Publisher('/light', Bool)
        self._BlueLightPublisher = rospy.Publisher('/bluelight', Bool)

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
         
        self.supply = -1
        self._widget.supply_lineEdit.setText('unknown')
        # init percept model
        
        
    def _on_tracker_supply(self, message):
        self.supply = message.data
        self._update_task_delegates.emit()

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

    

    def _on_update_task_delegates(self):
        if (self.supply == -1) :
            self._widget.supply_lineEdit.setText('unknown')
        else:
            self._widget.supply_lineEdit.setText(str(self.supply))
   
