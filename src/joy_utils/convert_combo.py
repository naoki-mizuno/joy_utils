from .combo import Combo
from .converter import Converter


class ConvertCombo(Combo):
    def __init__(self, rules, **kwargs):
        super(ConvertCombo, self).__init__(**kwargs)

        self._converter = Converter(rules)

    def convert_and_process(self, joy_msg):
        converted = self._converter.convert(joy_msg)
        return self.process(converted)
