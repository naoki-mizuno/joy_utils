import six

import collections
try:
    import numpy as np
except ImportError:
    np = None
import math


class Converter(object):
    def __init__(self, rules):
        self.rules = rules

    def get(self, joy_msg):
        """
        Normalizes the given Joy message following the conversion rule
        :param joy_msg:
        :return: Converted values
        :rtype: [float|bool|dict[float|bool]]
        """
        return self.convert(joy_msg, self.rules)

    @staticmethod
    def convert(joy_msg, rules):
        """
        :param joy_msg:
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
        If expr is a list or tuple, a list is returned by recursively
        calling Converter.eval.
        :param joy_msg:
        :param expr:
        :type expr: str|list|tuple|dict
        :return: the evaluated value
        """
        if not isinstance(expr, six.string_types) and isinstance(expr, collections.Iterable):
            return [Converter.eval(joy_msg, _expr) for _expr in expr]

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
