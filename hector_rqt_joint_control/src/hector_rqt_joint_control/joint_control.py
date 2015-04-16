import roslib
roslib.load_manifest('hector_rqt_joint_control')

import rospy
import math

from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Slot, QAbstractListModel, Qt
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QCheckBox, QLabel, QListWidget, QPushButton, QComboBox

from joint_control_widget import JointControlWidget

class JointControlDialog(Plugin):

    def __init__(self, context):
        super(JointControlDialog, self).__init__(context)
        self.setObjectName('JointControlDialog')

        self._parent = QWidget()
        self._widget = JointControlWidget(self._parent)
        
        context.add_widget(self._parent)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()

