import os
import rospy
import rospkg
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QEvent, QModelIndex, QObject, Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QShortcut, QTableWidgetItem, QWidget, QLCDNumber, QItemDelegate, QAbstractItemView, QColor, QFont
from rospy import Time
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class CO2Detection(QObject):
    _update_co2_color = Signal()
    _update_no_co2_color = Signal()

    def __init__(self, context):
        QObject.__init__(self, context)
        self.setObjectName('CO2Detection')
         # setup main widget
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('hector_co2_detection_plugin'), 'lib', 'CO2Detection.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CO2Detection')

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
        self._CO2Subscriber = rospy.Subscriber("/co2detected", Bool, self._on_co2_detected)
        
        # style settings
        self._co2_detected_color = QColor(0, 0, 0, 255)
        self._status_no_co2_style = "background-color: rgb(50, 255, 50);"
        self._status_co2_style = "background-color: rgb(255, 50, 50);"
        
        self._co2_font = QFont()
        self._co2_font.setBold(True)

       

        # Qt Signals
        #self.connect(self, QtCore.SIGNAL('setCO2Style(PyQt_PyObject)'), self._set_co2_style)
        self._update_co2_color.connect(self._set_co2_style)
        self._update_no_co2_color.connect(self._set_no_co2_style)

        self._widget.co2detectbutton.setText("Clear Air")
        self._update_no_co2_color.emit()
        
   

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

    def _on_co2_detected(self,msg):
        
        self._widget.co2detectbutton.setText("Warning: CO2 detected !!!" if msg.data else "Clear Air")
        #self.emit(QtCore.SIGNAL('setCO2Style(PyQt_PyObject)'), self._status_co2_style)
        if msg.data:
            self._update_co2_color.emit()
        else:
            self._update_no_co2_color.emit()

    def _set_co2_style(self):
        style_sheet_string = self._status_co2_style
        self._widget.co2detectbutton.setStyleSheet(style_sheet_string)

    def _set_no_co2_style(self):
        style_sheet_string = self._status_no_co2_style
        self._widget.co2detectbutton.setStyleSheet(style_sheet_string)
       
