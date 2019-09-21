from sensor_msgs.msg import Joy

import six

try:
    import numpy as np
except ImportError:
    np = None
import math


class Converter(object):
    def __init__(self, rules=None):
        self.rules = rules

    def get(self, joy_msg, keys=None):
        """
        Normalizes the given Joy message following the conversion rule
        If one value is given as the key, dict is returned.
        :param joy_msg:
        :type joy_msg: Joy
        :param keys: Keys in the conversion rule
        :type keys: [str,list[str]]
        :return: Converted values
        :rtype: [float|bool|dict[float|bool]]
        """
        if keys is None:
            # Convert using all known conversions
            return self.convert(joy_msg, self.rules)
        elif isinstance(keys, six.string_types):
            # Only one key was given
            return self.eval(joy_msg, self.rules[keys])

        normalized = {}
        for key in keys:
            expr = self.rules[key]
            val = self.eval(joy_msg, expr)
            normalized[key] = val

        return normalized

    @staticmethod
    def convert(joy_msg, rules):
        """
        :param joy_msg:
        :type joy_msg: Joy
        :param rules:
        :type rules: dict[str]
        :return:
        """
        normalized = {}
        for key, expr in rules.items():
            normalized[key] = Converter.eval(joy_msg, expr)
        return normalized

    @staticmethod
    def eval(joy_msg, expr):
        """
        Evaluates a given expression using values from the joy message
        For example, expression 'a1 + a3 if b0 else a5' evaluates to the
        result of 'a1 + a3' if b0 is high, otherwise to the value of a5.
        :param joy_msg:
        :type joy_msg: Joy
        :param expr:
        :type expr: str
        :return: the evaluated value
        """
        # Modules available for mathematical computation
        global_vars = {
            'm': np if np is not None else math,
            'math': math,
        }
        if np is not None:
            global_vars['np'] = np

        local_vars = {}
        for index, val in enumerate(joy_msg.axes):
            key = 'a{}'.format(index)
            local_vars[key] = val
        for index, val in enumerate(joy_msg.buttons):
            key = 'b{}'.format(index)
            local_vars[key] = bool(val)
        return eval(expr, global_vars, local_vars)
