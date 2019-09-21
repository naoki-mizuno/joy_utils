from joy_utils.converter import Converter
from sensor_msgs.msg import Joy

import pytest


class TestConverter(object):
    # noinspection PyAttributeOutsideInit
    def setup_method(self):
        self.joy = Joy()
        # 0, 0.1, 0.2, ... 0.9
        self.joy.axes = [float(i) / 10 for i in range(11)]
        # 0, 1, 0, 1, 0
        self.joy.buttons = [i % 2 for i in range(5)]

        self.conversion = {
            'sticks/left/lat': 'a0',
            'trigger/left': 'a2 * -0.5 + 0.5',
            'dpad/up': 'a7 > 0',
            'l1': 'b3',
            'l2': 'a2 < -0.1',
        }
        self.answer = {
            'sticks/left/lat': 0,
            'trigger/left': 0.4,
            'dpad/up': True,
            'l1': True,
            'l2': False,
        }

    def test_get(self):
        n = Converter(self.conversion)
        assert n.get(self.joy) == self.answer
        assert n.get(self.joy, 'l1') == self.answer['l1']
        assert n.get(self.joy, ('sticks/left/lat', 'dpad/up')) == {
            'sticks/left/lat': self.answer['sticks/left/lat'],
            'dpad/up': self.answer['dpad/up'],
        }

    def test_convert(self):
        assert Converter.convert(self.joy, self.conversion) == self.answer

    def test_eval(self):
        assert Converter.eval(self.joy, 'a2') == 0.2
        assert Converter.eval(self.joy, 'a9 + a1') == pytest.approx(1.0)
        assert Converter.eval(self.joy, 'a1 - a6') == pytest.approx(-0.5)
        assert Converter.eval(self.joy, 'a1 * a8') == pytest.approx(0.08)
        assert Converter.eval(self.joy, 'a8 / a2') == pytest.approx(4.0)
        # Use builtin functions
        assert Converter.eval(self.joy, 'abs(a1 - a8)') == pytest.approx(0.7)
        # Use whatever math library available
        assert Converter.eval(self.joy, 'm.log10(a1)') == -1.0
        # Use the math library
        assert Converter.eval(self.joy, 'math.log10(a1)') == -1.0
        # Use numpy (if installed on system)
        try:
            import numpy
            assert Converter.eval(self.joy, 'np.log10(a1)') == -1.0
        except ImportError:
            pass

        # Boolean logic
        assert Converter.eval(self.joy, 'b0 and b1') is False
        assert Converter.eval(self.joy, 'b0 or b1') is True
        assert Converter.eval(self.joy, 'b0 & b1') is False
        assert Converter.eval(self.joy, 'b0 | b1') is True
        assert Converter.eval(self.joy, 'b0 ^ b1') is True

        # Conditionals
        assert Converter.eval(self.joy, 'a2 if b0 else a6') == 0.6
        assert Converter.eval(self.joy, 'a2 if b1 else a6') == 0.2
