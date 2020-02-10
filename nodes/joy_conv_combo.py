#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from joy_utils import ConvertCombo


class JoyConvCombo(object):
    def __init__(self):
        rules = {
            'foo': 'b0',
            'bar': 'b1',
        }
        combos = [
            ConvertCombo.make_combo('foo'),
            ConvertCombo.make_combo('foo', 'bar'),
        ]
        self._conv_combo = ConvertCombo(rules,
                                        combos=combos,
                                        timeout_key=0.05,
                                        timeout_combo=0.3)

        self._sub_joy = rospy.Subscriber('joy', Joy, self.cb_joy, queue_size=1)

        self._timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.cb_timer)

    def cb_joy(self, msg):
        # Method returns: high, up, down
        self._conv_combo.convert_and_process(msg)

    def cb_timer(self, _):
        if self._conv_combo.triggered_combo is not None:
            rospy.loginfo(', then '.join([' & '.join(key) for key in self._conv_combo.triggered_combo]))
            self._conv_combo.clear()


def main():
    rospy.init_node('joy_conv_combo')

    JoyConvCombo()

    rospy.spin()


if __name__ == '__main__':
    main()
