import os
import sys
import unittest

import numpy as np

SRC_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from perception.color_segmenter import ColorRuleSegmenter


class ColorRuleSegmenterTest(unittest.TestCase):
    def setUp(self):
        self.segmenter = ColorRuleSegmenter({
            "pick_object": {
                "aliases": ["pick object", "red cube"],
                "dominant_channel": "red",
                "min_value": 100,
                "dominance_ratio": 1.5,
            },
            "place_target": {
                "aliases": ["green target"],
                "dominant_channel": "green",
                "min_value": 90,
                "dominance_ratio": 1.35,
            },
        })

    def test_aliases_select_expected_pixels(self):
        image = np.array(
            [[[200, 20, 10], [20, 180, 30], [100, 100, 100]]], dtype=np.uint8
        )
        np.testing.assert_array_equal(
            self.segmenter("red cube", image), [[True, False, False]]
        )
        np.testing.assert_array_equal(
            self.segmenter("place_target", image), [[False, True, False]]
        )

    def test_unknown_object_lists_available_names(self):
        with self.assertRaisesRegex(ValueError, "pick_object"):
            self.segmenter("blue cup", np.zeros((1, 1, 3), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
