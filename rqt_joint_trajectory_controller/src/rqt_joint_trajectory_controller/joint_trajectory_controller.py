#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout

from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .double_editor import DoubleEditor
from .joint_limits_urdf import get_joint_limits
from .update_combo import update_combo

# TODO:
# - Better UI suppor for continuous joints (see DoubleEditor TODO)
# - Can we load controller joints faster?, it's currently pretty slow
# - If URDF is reloaded, allow to reset the whole plugin?
# - Allow to configure:
#   - URDF location
#   - Command publishing and state update frequency
#   - Controller manager and jtc monitor frequency
#   - Min trajectory duration
# - Fail gracefully when the URDF or some other requisite is not set
# - Could users confuse the enable/disable button with controller start/stop
#   (in a controller manager sense)?
# - Better decoupling between model and view
# - Tab order is wrong. For the record, this did not work:
#   QWidget.setTabOrder(self._widget.controller_group,
#                       self._widget.joint_group)
#   QWidget.setTabOrder(self._widget.joint_group,
#                       self._widget.speed_scaling_group)

# NOTE:
# Controller enable/disable icons are in the public domain, and available here:
# freestockphotos.biz/photos.php?c=all&o=popular&s=0&lic=all&a=all&set=powocab_u2


class JointTrajectoryController(Plugin):
    """
    Graphical frontend for a C{JointTrajectoryController}.

    There are two modes for interacting with a controller:
        1. B{Monitor mode} Joint displays are updated with the state reported
          by the controller. This is a read-only mode and does I{not} send
          control commands. Every time a new controller is selected, it starts
          in monitor mode for safety reasons.

        2. B{Control mode} Joint displays update the control command that is
        sent to the controller. Commands are sent periodically evan if the
        displays are not being updated by the user.

    To control the aggressiveness of the motions, the maximum speed of the
    sent commands can be scaled down using the C{Speed scaling control}

    This plugin is able to detect and keep track of all active controller
    managers, as well as the JointTrajectoryControllers that are I{running}
    in each controller manager.
    For a controller to be compatible with this plugin, it must comply with
    the following requisites:
        - The controller type contains the C{JointTrajectoryController}
        substring, e.g., C{position_controllers/JointTrajectoryController}
        - The controller exposes the C{command} and C{state} topics in its
        ROS interface.

    Additionally, there must be a URDF loaded with a valid joint limit
    specification, namely position (when applicable) and velocity limits.

    A reference implementation of the C{JointTrajectoryController} is
    available in the C{joint_trajectory_controller} ROS package.
    """
    _cmd_pub_freq = 10.0  # Hz
    _widget_update_freq = 30.0  # Hz
    _ctrlrs_update_freq = 1  # Hz
    _min_traj_dur = 5.0 / _cmd_pub_freq  # Minimum trajectory duration

    jointStateChanged = Signal([dict])

    def __init__(self, context):
        super(JointTrajectoryController, self).__init__(context)
        self.setObjectName('JointTrajectoryController')

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_joint_trajectory_controller'),
                               'resource',
                               'joint_trajectory_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('JointTrajectoryControllerUi')
        ns = rospy.get_namespace()[1:-1]
        self._widget.controller_group.setTitle('ns: ' + ns)

        # Setup speed scaler
        speed_scaling = DoubleEditor(1.0, 100.0)
        speed_scaling.spin_box.setSuffix('%')
        speed_scaling.spin_box.setValue(50.0)
        speed_scaling.spin_box.setDecimals(0)
        speed_scaling.setEnabled(False)
        self._widget.speed_scaling_layout.addWidget(speed_scaling)
        self._speed_scaling_widget = speed_scaling
        speed_scaling.valueChanged.connect(self._on_speed_scaling_change)
        self._on_speed_scaling_change(speed_scaling.value())

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._jtc_name = []  # Name of selected joint trajectory controller
        self._cm_ns = []  # Namespace of the selected controller manager
        self._joint_pos = {}  # name->pos map for joints of selected controller
        self._joint_names = []  # Ordered list of selected controller joints
        self._robot_joint_limits = {} # Lazily evaluated on first use

        # Timer for sending commands to active controller
        self._update_cmd_timer = QTimer(self)
        self._update_cmd_timer.setInterval(int(1000.0 / self._cmd_pub_freq))
        self._update_cmd_timer.timeout.connect(self._update_cmd_cb)

        # Timer for updating the joint widgets from the controller state
        self._update_act_pos_timer = QTimer(self)
        self._update_act_pos_timer.setInterval(int(1000.0 /
                                               self._widget_update_freq))
        self._update_act_pos_timer.timeout.connect(self._update_joint_widgets)

        # Timer for controller manager updates
        self._list_cm = ControllerManagerLister()
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(int(1000.0 /
                                               self._ctrlrs_update_freq))
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_jtc_list_timer = QTimer(self)
        self._update_jtc_list_timer.setInterval(int(1000.0 /
                                                self._ctrlrs_update_freq))
        self._update_jtc_list_timer.timeout.connect(self._update_jtc_list)
        self._update_jtc_list_timer.start()

        # Signal connections
        w = self._widget
        w.enable_button.toggled.connect(self._on_jtc_enabled)
        w.jtc_combo.currentIndexChanged[str].connect(self._on_jtc_change)
        w.cm_combo.currentIndexChanged[str].connect(self._on_cm_change)

        self._cmd_pub = None    # Controller command publisher
        self._state_sub = None  # Controller state subscriber

        self._list_controllers = None

    def shutdown_plugin(self):
        self._update_cmd_timer.stop()
        self._update_act_pos_timer.stop()
        self._update_cm_list_timer.stop()
        self._update_jtc_list_timer.stop()
        self._unregister_state_sub()
        self._unregister_cmd_pub()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('cm_ns', self._cm_ns)
        instance_settings.set_value('jtc_name', self._jtc_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session's controller_manager, if present
        self._update_cm_list()
        cm_ns = instance_settings.value('cm_ns')
        cm_combo = self._widget.cm_combo
        cm_list = [cm_combo.itemText(i) for i in range(cm_combo.count())]
        try:
            idx = cm_list.index(cm_ns)
            cm_combo.setCurrentIndex(idx)
            # Resore last session's controller, if running
            self._update_jtc_list()
            jtc_name = instance_settings.value('jtc_name')
            jtc_combo = self._widget.jtc_combo
            jtc_list = [jtc_combo.itemText(i) for i in range(jtc_combo.count())]
            try:
                idx = jtc_list.index(jtc_name)
                jtc_combo.setCurrentIndex(idx)
            except (ValueError):
                pass
        except (ValueError):
            pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget
        # title bar
        # Usually used to open a modal configuration dialog

    def _update_cm_list(self):
        update_combo(self._widget.cm_combo, self._list_cm())

    def _update_jtc_list(self):
        # Clear controller list if no controller information is available
        if not self._list_controllers:
            self._widget.jtc_combo.clear()
            return

        # List of running controllers with a valid joint limits specification
        # for _all_ their joints
        running_jtc = self._running_jtc_info()
        if running_jtc and not self._robot_joint_limits:
            self._robot_joint_limits = get_joint_limits()  # Lazy evaluation
        valid_jtc = []
        for jtc_info in running_jtc:
            has_limits = all(name in self._robot_joint_limits
                             for name in _jtc_joint_names(jtc_info))
            if has_limits:
                valid_jtc.append(jtc_info);
        valid_jtc_names = [data.name for data in valid_jtc]

        # Update widget
        update_combo(self._widget.jtc_combo, sorted(valid_jtc_names))

    def _on_speed_scaling_change(self, val):
        self._speed_scale = val / self._speed_scaling_widget.slider.maximum()

    def _on_joint_state_change(self, actual_pos):
        assert(len(actual_pos) == len(self._joint_pos))
        for name in actual_pos.keys():
            self._joint_pos[name]['position'] = actual_pos[name]

    def _on_cm_change(self, cm_ns):
        self._cm_ns = cm_ns
        if cm_ns:
            self._list_controllers = ControllerLister(cm_ns)
            # NOTE: Clear below is important, as different controller managers
            # might have controllers with the same name but different
            # configurations. Clearing forces controller re-discovery
            self._widget.jtc_combo.clear()
            self._update_jtc_list()
        else:
            self._list_controllers = None

    def _on_jtc_change(self, jtc_name):
        self._unload_jtc()
        self._jtc_name = jtc_name
        if self._jtc_name:
            self._load_jtc()

    def _on_jtc_enabled(self, val):
        # Don't allow enabling if there are no controllers selected
        if not self._jtc_name:
            self._widget.enable_button.setChecked(False)
            return

        # Enable/disable joint displays
        for joint_widget in self._joint_widgets():
            joint_widget.setEnabled(val)

        # Enable/disable speed scaling
        self._speed_scaling_widget.setEnabled(val)

        if val:
            # Widgets send desired position commands to controller
            self._update_act_pos_timer.stop()
            self._update_cmd_timer.start()
        else:
            # Controller updates widgets with actual position
            self._update_cmd_timer.stop()
            self._update_act_pos_timer.start()

    def _load_jtc(self):
        # Initialize joint data corresponding to selected controller
        running_jtc = self._running_jtc_info()
        self._joint_names = next(_jtc_joint_names(x) for x in running_jtc
                                 if x.name == self._jtc_name)
        for name in self._joint_names:
            self._joint_pos[name] = {}

        # Update joint display
        try:
            layout = self._widget.joint_group.layout()
            for name in self._joint_names:
                limits = self._robot_joint_limits[name]
                joint_widget = DoubleEditor(limits['min_position'],
                                            limits['max_position'])
                layout.addRow(name, joint_widget)
                # NOTE: Using partial instead of a lambda because lambdas
                # "will not evaluate/look up the argument values before it is
                # effectively called, breaking situations like using a loop
                # variable inside it"
                from functools import partial
                par = partial(self._update_single_cmd_cb, name=name)
                joint_widget.valueChanged.connect(par)
        except:
            # TODO: Can we do better than swallow the exception?
            from sys import exc_info
            print('Unexpected error:', exc_info()[0])

        # Enter monitor mode (sending commands disabled)
        self._on_jtc_enabled(False)

        # Setup ROS interfaces
        jtc_ns = _resolve_controller_ns(self._cm_ns, self._jtc_name)
        state_topic = jtc_ns + '/state'
        cmd_topic = jtc_ns + '/command'
        self._state_sub = rospy.Subscriber(state_topic,
                                           JointTrajectoryControllerState,
                                           self._state_cb,
                                           queue_size=1)
        self._cmd_pub = rospy.Publisher(cmd_topic,
                                        JointTrajectory,
                                        queue_size=1)

        # Start updating the joint positions
        self.jointStateChanged.connect(self._on_joint_state_change)

    def _unload_jtc(self):
        # Stop updating the joint positions
        try:
            self.jointStateChanged.disconnect(self._on_joint_state_change)
        except:
            pass

        # Reset ROS interfaces
        self._unregister_state_sub()
        self._unregister_cmd_pub()

        # Clear joint widgets
        # NOTE: Implementation is a workaround for:
        # https://bugreports.qt-project.org/browse/QTBUG-15990 :(
					
        layout = self._widget.joint_group.layout()
        if layout is not None:
            while layout.count():
                layout.takeAt(0).widget().deleteLater()
            # Delete existing layout by reparenting to temporary
            QWidget().setLayout(layout)
        self._widget.joint_group.setLayout(QFormLayout())

        # Reset joint data
        self._joint_names = []
        self._joint_pos = {}

        # Enforce monitor mode (sending commands disabled)
        self._widget.enable_button.setChecked(False)

    def _running_jtc_info(self):
        from controller_manager_msgs.utils\
            import filter_by_type, filter_by_state

        controller_list = self._list_controllers()
        jtc_list = filter_by_type(controller_list,
                                  'JointTrajectoryController',
                                  match_substring=True)
        running_jtc_list = filter_by_state(jtc_list, 'running')
        return running_jtc_list

    def _unregister_cmd_pub(self):
        if self._cmd_pub is not None:
            self._cmd_pub.unregister()
            self._cmd_pub = None

    def _unregister_state_sub(self):
        if self._state_sub is not None:
            self._state_sub.unregister()
            self._state_sub = None

    def _state_cb(self, msg):
        actual_pos = {}
        for i in range(len(msg.joint_names)):
            joint_name = msg.joint_names[i]
            joint_pos = msg.actual.positions[i]
            actual_pos[joint_name] = joint_pos
        self.jointStateChanged.emit(actual_pos)

    def _update_single_cmd_cb(self, val, name):
        self._joint_pos[name]['command'] = val

    def _update_cmd_cb(self):
        dur = []
        traj = JointTrajectory()
        traj.joint_names = self._joint_names
        point = JointTrajectoryPoint()
        for name in traj.joint_names:
            pos = self._joint_pos[name]['position']
            cmd = pos
            try:
                cmd = self._joint_pos[name]['command']
            except (KeyError):
                pass
            max_vel = self._robot_joint_limits[name]['max_velocity']
            dur.append(max(abs(cmd - pos) / max_vel, self._min_traj_dur))
            point.positions.append(cmd)
        point.time_from_start = rospy.Duration(max(dur) / self._speed_scale)
        traj.points.append(point)

        self._cmd_pub.publish(traj)

    def _update_joint_widgets(self):
        joint_widgets = self._joint_widgets()
        for id in range(len(joint_widgets)):
            joint_name = self._joint_names[id]
            try:
                joint_pos = self._joint_pos[joint_name]['position']
                joint_widgets[id].setValue(joint_pos)
            except (KeyError):
                pass  # Can happen when first connected to controller

    def _joint_widgets(self):  # TODO: Cache instead of compute every time?
        widgets = []
        layout = self._widget.joint_group.layout()
        for row_id in range(layout.rowCount()):
            widgets.append(layout.itemAt(row_id,
                                         QFormLayout.FieldRole).widget())
        return widgets

def _jtc_joint_names(jtc_info):
    # NOTE: We assume that there is at least one hardware interface that
    # claims resources (there should be), and the resource list is fetched
    # from the first available interface
    return jtc_info.claimed_resources[0].resources

def _resolve_controller_ns(cm_ns, controller_name):
    """
    Resolve a controller's namespace from that of the controller manager.
    Controllers are assumed to live one level above the controller
    manager, e.g.

        >>> _resolve_controller_ns('/path/to/controller_manager', 'foo')
        '/path/to/foo'

    In the particular case in which the controller manager is not
    namespaced, the controller is assumed to live in the root namespace

        >>> _resolve_controller_ns('/', 'foo')
        '/foo'
        >>> _resolve_controller_ns('', 'foo')
        '/foo'
    @param cm_ns Controller manager namespace (can be an empty string)
    @type cm_ns str
    @param controller_name Controller name (non-empty string)
    @type controller_name str
    @return Controller namespace
    @rtype str
    """
    assert(controller_name)
    ns = cm_ns.rsplit('/', 1)[0]
    if ns != '/':
        ns += '/'
    ns += controller_name
    return ns
