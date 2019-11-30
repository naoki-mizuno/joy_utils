from joy_utils.combo import Combo


def fs(*args):
    return frozenset(args)


class TestCombo(object):
    # noinspection PyAttributeOutsideInit
    def setup_method(self):
        # Description
        #   - String representation
        #   - String interpreted to combo
        #   - Combo squashed
        self.combo_definitions = [
            # Single button press/release (denoted as P/R)
            (
                ['b0:d', 'b0:u'],
                (fs('b0:d'), fs('b0:u')),
                (fs('b0'),),
            ),

            # b0 P/R -> b1 P/R
            (
                ['b0', 'b1:d', 'b1:u'],
                (fs('b0:d'), fs('b0:u'), fs('b1:d'), fs('b1:u')),
                (fs('b0'), fs('b1')),
            ),

            # b0 press -> b1 press -> b0 release -> b1 release
            (
                ['b0:d', 'b1:d', 'b0:u', 'b1:u'],
                (fs('b0:d'), fs('b1:d'), fs('b0:u'), fs('b1:u')),
                (fs('b0:d'), fs('b1:d'), fs('b0:u'), fs('b1:u')),
            ),

            # b0 press -> b1 P/R -> b0 release
            (
                ['b0:d', 'b1', 'b0:u'],
                (fs('b0:d'), fs('b1:d'), fs('b1:u'), fs('b0:u')),
                (fs('b0:d'), fs('b1'), fs('b0:u')),
            ),
        ]

    def test_make_combo(self):
        for string, _, squashed in self.combo_definitions:
            assert Combo.make_combo(string) == squashed
        # More cases
        assert Combo.make_combo('b0') == (fs('b0'),)

    def test_squash_key(self):
        k1 = {'b0:d', 'b0:u'}
        assert Combo._squash_key_(k1) == {'b0'}
        k2 = {'b0'}
        assert Combo._squash_key_(k2) == {'b0'}
        k3 = {'b0:d', 'b0:u', 'b1:d', 'b1:u'}
        assert Combo._squash_key_(k3) == {'b0', 'b1'}

    def test_squash_combo(self):
        for _, orig_combo, squashed_combo in self.combo_definitions:
            assert Combo._squash_combo_(orig_combo) == squashed_combo
