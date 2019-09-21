#!/usr/bin/env python

from joy_utils.converter import Converter

import rospy
from sensor_msgs.msg import Joy


class JoyConv(object):
    def __init__(self, rules):
        self._converter = Converter(rules)
        self._pub = rospy.Publisher('joy', Joy, queue_size=1)

        custom_keys = [key for key in rules if key[0] not in list('ab')]
        if len(custom_keys) != 0:
            fmt = 'Custom keys will be ignored: {}'
            rospy.logwarn(fmt.format(', '.join(custom_keys)))

    def cb_joy(self, joy_msg):
        converted = self._converter.get(joy_msg)

        new_joy = Joy()
        for key, val in converted.items():
            key = key.lower()
            if key.startswith('a'):
                arr = new_joy.axes
            elif key.startswith('b'):
                arr = new_joy.buttons
            else:
                # Key not in the expected format. No problem, simply ignore it
                continue
            # Expected key format is aXX or bYY where XX and YY are integers
            index = int(key[1:])
            JoyConv._set_(arr, index, val)
        self._pub.publish(new_joy)

    @staticmethod
    def _set_(arr, index, val, fill_val=0):
        """
        Expands array if necessary (filling in zeros) and sets the given value
        :param arr:
        :type arr: list
        :param index:
        :type index: int
        :param val:
        :param fill_val: value used to fill in the gap
        :return:
        """
        while len(arr) <= index:
            arr.append(fill_val)
        arr[index] = val


def main():
    rospy.init_node('joy_conv')

    rules = rospy.get_param('~rules', None)

    converter = JoyConv(rules)
    rospy.Subscriber('joy_in',
                     Joy,
                     converter.cb_joy,
                     queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
