import rospy
from sensor_msgs.msg import Joy

import copy
import six


class Combo(object):
    def __init__(self,
                 combos=None,
                 timeout_key=0.05,
                 timeout_combo=0.3,
                 data_to_dict=None):
        """
        Terminology:
          Key: one button press or a group of buttons pressed simultaneously
          Combo: sequence of keys

        :param combos: all available combos
        :type combos: collections.Iterable[tuple[frozenset[str]]]
        :param timeout_key: timeout to consider button presses simultaneous
        :param timeout_combo: timeout to consider combo input to be completed
        :param data_to_dict: method to convert arbitrary data to dict
        """
        self._combo_definitions = combos
        self._timeout_key = timeout_key
        self._timeout_combo = timeout_combo
        # Method to convert arbitrary data to dict[button_name, value]
        if data_to_dict is None:
            self._data_to_dict = self._joy_msg_to_dict_
        else:
            self._data_to_dict = data_to_dict

        self._prev_high_buttons = set()

        # Contains the triggered combo
        self.triggered_combo = None
        self._current_key = set()
        self._current_combo = []

        self._timer_key = None
        self._timer_combo = None

    def process(self, data):
        """
        Processes the input data and update combo
        Note that a combo is not completed during this method call because we
        need to wait for the combo timeout after the last key was pressed
        (i.e. this method was called). Check whether the value of
        Combo.triggered_combo is None.
        :return: name of buttons that are high, released, and pressed
        :rtype: tuple[set[str], set[str], set[str]]
        """
        high_buttons = self._get_high_buttons_(data)

        up = self._prev_high_buttons - high_buttons
        down = high_buttons - self._prev_high_buttons

        self._add_to_key_(up, down)

        self._prev_high_buttons = high_buttons

        return high_buttons, up, down

    def ongoing_combo(self):
        return copy.deepcopy(self._current_combo)

    def clear(self):
        self.triggered_combo = None
        self._current_key.clear()
        self._current_combo = []

    def _get_high_buttons_(self, data):
        """
        Creates a set of buttons that have high values (i.e. 1)
        :param data: dict of button name and value
        :return: name of the buttons that are high
        :rtype: set[str]
        """
        if isinstance(data, dict):
            msg_dict = data
        else:
            msg_dict = self._data_to_dict(data)

        high_buttons = set()
        for button_name, value in msg_dict.items():
            if value:
                high_buttons.add(str(button_name))
        return high_buttons

    def _add_to_key_(self, up, down):
        # No key was pressed or released
        if len(up.union(down)) == 0:
            return

        for button in up:
            self._current_key.add(button + ':u')
        for button in down:
            self._current_key.add(button + ':d')

        if self._timer_key is not None:
            self._timer_key.shutdown()
        self._timer_key = rospy.Timer(
            rospy.Duration.from_sec(self._timeout_key),
            self._add_to_combo_,
            oneshot=True,
        )

    def _add_to_combo_(self, _):
        # print('Pressed: {}'.format(self._current_key))
        self._current_combo.append(frozenset(self._current_key))
        self._current_key.clear()

        if self._timer_combo is not None:
            self._timer_combo.shutdown()
        self._timer_combo = rospy.Timer(
            rospy.Duration.from_sec(self._timeout_combo),
            self._set_combo_,
            oneshot=True,
        )

    def _set_combo_(self, _):
        combo = self._squash_combo_(self._current_combo)

        # print('Combo {}'.format(combo))
        if self._combo_definitions is None or combo in self._combo_definitions:
            self.triggered_combo = copy.deepcopy(combo)
        else:
            self.clear()

    @staticmethod
    def _squash_combo_(combo):
        """
        Squash consecutive press/release of the same key into one key
        For example, squashing [button1:d, button2:d, button2:u, button1:u] becomes
        [button1:d, button2, button1:u].
        :param combo:
        :type combo: collections.Sequence[collections.Set[str]]
        :return:
        """
        squashed_combo = []

        last_key_was_squashed = False
        # Squash back-to-back press/release
        for i, key_set in enumerate(combo[:-1]):
            next_key_set = combo[i + 1]

            # If last key was squashed, we need to skip over this key because
            # it is already squashed into the previous key
            if last_key_was_squashed:
                last_key_was_squashed = False
                continue

            # Only squash key that was pressed and then released
            all_down = all([button.endswith(':d') for button in key_set])
            all_up = all([button.endswith(':u') for button in next_key_set])
            if not all_down or not all_up:
                squashed_combo.append(Combo._squash_key_(key_set))
                continue

            buttons = set([b.rsplit(':', 1)[0] for b in key_set])
            next_buttons = set([b.rsplit(':', 1)[0] for b in next_key_set])

            # Back-to-back press/release
            last_key_was_squashed = buttons == next_buttons
            if last_key_was_squashed:
                squashed_combo.append(frozenset(buttons))
            # Squash key like (3:u, 3:d) to (3)
            # but leave key like (3:u, circle:u) as is
            else:
                squashed_combo.append(Combo._squash_key_(key_set))

        if not last_key_was_squashed:
            squashed_combo.append(Combo._squash_key_(combo[-1]))

        return tuple(squashed_combo)

    @staticmethod
    def _squash_key_(key):
        """
        :param key:
        :type key: collections.Set[str]
        :return:
        """
        combined = set()
        for button_with_suffix in key:
            split_tokens = button_with_suffix.rsplit(':', 1)
            # No colon was found (i.e. no suffix was present)
            if len(split_tokens) == 1:
                combined.add(button_with_suffix)
                continue

            button, suffix = split_tokens
            opposite_suffix = 'd' if suffix == 'u' else 'u'

            # If both up and down are in the same key, use name of button
            if button + ':' + opposite_suffix in key:
                combined.add(button)
            # Otherwise, append suffix (:u or :d) to indicate up/down
            else:
                combined.add(button_with_suffix)
        return frozenset(combined)

    @staticmethod
    def _joy_msg_to_dict_(joy_msg):
        """
        Converts a sensor_msgs/Joy message to dict
        :type joy_msg: Joy
        :return dict of button indices (in string) and values
        """
        if not isinstance(joy_msg, Joy):
            name = joy_msg.__class__.__name__
            fmt = 'Expected Joy instance but got {} instead'
            rospy.logwarn_throttle(5, fmt.format(name))

        msg_dict = {}
        for i, value in enumerate(joy_msg.buttons):
            msg_dict[str(i)] = value
        return msg_dict

    @staticmethod
    def make_combo(*args):
        """
        Creates a combo from the given arguments
        :param args:
        :return:
        """
        if len(args) == 0:
            raise ValueError('At least one argument must be passed')
        elif len(args) > 1:
            # E.g. make_combo('b0', 'b1', 'b2')
            combo = args
        elif isinstance(args[0], six.string_types):
            # E.g. make_combo('b0')
            combo = [args[0]]
        else:
            # E.g. make_combo(['b0', 'b1])
            combo = args

        ret = []
        for key in combo:
            if isinstance(key, six.string_types):
                key = [key]
            ret.append(frozenset(key))
        return Combo._squash_combo_(ret)
