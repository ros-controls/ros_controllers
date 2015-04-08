#!/usr/bin/env python

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

import xml.dom.minidom
from math import pi

import rospy


def get_joint_limits(key='robot_description', use_smallest_joint_limits=True):
    use_small = use_smallest_joint_limits
    use_mimic = True

    # Code inspired on the joint_state_publisher package by David Lu!!!
    # https://github.com/ros/robot_model/blob/indigo-devel/
    # joint_state_publisher/joint_state_publisher/joint_state_publisher
    description = rospy.get_param(key)
    robot = xml.dom.minidom.parseString(description)\
        .getElementsByTagName('robot')[0]
    free_joints = {}
    dependent_joints = {}

    # Find all non-fixed joints
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'joint':
            jtype = child.getAttribute('type')
            if jtype == 'fixed':
                continue
            name = child.getAttribute('name')
            if jtype == 'continuous':
                minval = -pi
                maxval = pi
            else:
                try:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))
                except:
                    continue
            try:
                maxvel = float(limit.getAttribute('velocity'))
            except:
                continue
            safety_tags = child.getElementsByTagName('safety_controller')
            if use_small and len(safety_tags) == 1:
                tag = safety_tags[0]
                if tag.hasAttribute('soft_lower_limit'):
                    minval = max(minval,
                                 float(tag.getAttribute('soft_lower_limit')))
                if tag.hasAttribute('soft_upper_limit'):
                    maxval = min(maxval,
                                 float(tag.getAttribute('soft_upper_limit')))

            mimic_tags = child.getElementsByTagName('mimic')
            if use_mimic and len(mimic_tags) == 1:
                tag = mimic_tags[0]
                entry = {'parent': tag.getAttribute('joint')}
                if tag.hasAttribute('multiplier'):
                    entry['factor'] = float(tag.getAttribute('multiplier'))
                if tag.hasAttribute('offset'):
                    entry['offset'] = float(tag.getAttribute('offset'))

                dependent_joints[name] = entry
                continue

            if name in dependent_joints:
                continue

            joint = {'min_position': minval, 'max_position': maxval}
            joint["has_position_limits"] = jtype != 'continuous'
            joint['max_velocity'] = maxvel
            free_joints[name] = joint
    return free_joints
