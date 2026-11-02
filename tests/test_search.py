"""纯逻辑单测：搜索航线与目标簿记（不需要 ROS）。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

from autofly.search import (TargetBook, lawnmower, lane_spacing,  # noqa: E402
                            nearest, order_by_travel)


class TestLawnmower(unittest.TestCase):
    def test_keeps_order_and_dedupes(self):
        lanes = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        self.assertEqual(lawnmower(lanes), [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])

    def test_lane_spacing(self):
        self.assertAlmostEqual(lane_spacing([(0.0, 0), (0.0, 4), (2.6, 4), (2.6, 0)]), 2.6)

    def test_lane_spacing_single(self):
        self.assertEqual(lane_spacing([(1.0, 1.0)]), 0.0)


class TestTargetBook(unittest.TestCase):
    def setUp(self):
        self.book = TargetBook(("tent", "pillbox", "bridge", "panzer"))

    def test_add_and_best(self):
        self.assertTrue(self.book.add("tent", 1.0, 2.0, 0.9))
        self.assertTrue(self.book.add("bridge", 3.0, 4.0, 0.5))
        self.assertEqual(len(self.book), 2)
        best = self.book.best(2)
        self.assertEqual(best[0][0], "tent")           # 置信度高者在前
        self.assertAlmostEqual(best[0][1], 1.0)

    def test_reject_unknown_class(self):
        self.assertFalse(self.book.add("red_cross", 1.0, 1.0, 0.99))
        self.assertEqual(len(self.book), 0)

    def test_reject_outlier(self):
        self.assertFalse(self.book.add("tent", 99.0, 1.0, 0.9))
        self.assertFalse(self.book.add("tent", float("nan"), 1.0, 0.9))
        self.assertEqual(len(self.book), 0)

    def test_dedupe_by_class_keeps_higher_conf(self):
        self.book.add("tent", 1.0, 1.0, 0.5)
        self.book.add("tent", 2.0, 2.0, 0.9)
        self.book.add("tent", 3.0, 3.0, 0.4)           # 置信度更低，忽略
        t = self.book.get("tent")
        self.assertAlmostEqual(t[1], 2.0)
        self.assertAlmostEqual(t[3], 0.9)

    def test_best_limits(self):
        for i, c in enumerate(("tent", "pillbox", "bridge", "panzer")):
            self.book.add(c, float(i), 0.0, 0.1 * (i + 1))
        self.assertEqual(len(self.book.best(3)), 3)
        self.assertEqual(len(self.book.best(10)), 4)

    def test_as_dict_rounds(self):
        self.book.add("tent", 1.23456, 2.34567, 0.9)
        self.assertEqual(self.book.as_dict()["tent"], (1.23, 2.35))


class TestOrdering(unittest.TestCase):
    def test_nearest(self):
        pts = [(0.0, 0.0), (5.0, 0.0), (1.0, 0.0)]
        self.assertEqual(nearest(pts, (0.2, 0.0)), (0.0, 0.0))

    def test_empty_nearest(self):
        self.assertIsNone(nearest([], (0.0, 0.0)))

    def test_travel_order(self):
        pts = [(5.0, 0.0), (1.0, 0.0), (3.0, 0.0)]
        order = order_by_travel(pts, (0.0, 0.0))
        self.assertEqual(order, [(1.0, 0.0), (3.0, 0.0), (5.0, 0.0)])

    def test_travel_order_no_duplicates(self):
        pts = [(1.0, 0.0), (2.0, 0.0)]
        order = order_by_travel(pts, (1.0, 0.0))
        self.assertEqual(len(order), 2)
        self.assertEqual(sorted(order), sorted(pts))


if __name__ == "__main__":
    unittest.main(verbosity=2)
