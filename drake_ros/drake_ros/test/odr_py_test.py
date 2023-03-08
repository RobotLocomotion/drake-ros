"""
This tests for ODR violations in how Python bindings are wrapped. GeometryId is
used because it has global state. The next GeometryId is always one previous
the last one. If the next Id in Python is not exactly one more than the
previous Id requested via C++ then it means there are multiple definitions.

This strategy is similar to Drake's odr_test.py
"""

import unittest

from pydrake.geometry import GeometryId

from drake_ros._cc._test import NextGeometryId


class OdrTest(unittest.TestCase):
    def test_odr(self):
        id_via_pydrake = GeometryId.get_new_id()
        id_via_bespoke_bindings = NextGeometryId()
        self.assertEqual(
            id_via_bespoke_bindings.get_value(),
            id_via_pydrake.get_value() + 1,
            "ODR violation!",
        )


if __name__ == '__main__':
    unittest.main()
