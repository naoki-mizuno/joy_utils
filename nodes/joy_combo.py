#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from joy_utils.combo import Combo


def cb_timer(combo_manager, combo_dict):
    # Type of triggered: tuple[frozenset[str]]
    triggered = combo_manager.triggered_combo
    if triggered is not None:
        if triggered in combo_dict:
            rospy.loginfo(combo_dict[triggered])
        else:
            buttons = []
            for key in triggered:
                fmt = '({})' if len(key) > 1 else '{}'
                buttons.append(fmt.format(', '.join(sorted(key))))
            rospy.loginfo(', '.join(buttons))
        combo_manager.clear()


def main():
    rospy.init_node('joy_combo')

    timeout_key = rospy.get_param('~timeout/key', 0.05)
    timeout_combo = rospy.get_param('~timeout/combo', 0.3)

    combo_dict = {}
    for definition in rospy.get_param('~combos'):
        combo = definition['combo']
        to_print = definition['print']

        dict_key = Combo.make_combo(combo)
        combo_dict[dict_key] = to_print

    c = Combo(combo_dict.keys(),
              timeout_key=timeout_key,
              timeout_combo=timeout_combo)
    cb_timer.combo_handler = c

    rospy.Subscriber('joy', Joy, c.process, queue_size=1)
    rospy.Timer(rospy.Duration.from_sec(0.1), lambda _: cb_timer(c, combo_dict))

    rospy.spin()


if __name__ == '__main__':
    main()
