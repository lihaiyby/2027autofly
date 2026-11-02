"""配置加载单测（不需要 ROS / 不需要 PyYAML 也能跑）。"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))

from autofly.config import MissionConfig  # noqa: E402


class TestConfigDefaults(unittest.TestCase):
    def test_defaults(self):
        cfg = MissionConfig()
        self.assertAlmostEqual(cfg.search_z, 2.4)
        self.assertAlmostEqual(cfg.drop_z, 2.3)
        self.assertEqual(cfg.max_targets, 3)
        self.assertEqual(len(cfg.corridor), 11)
        self.assertIn("tent", cfg.want_classes)
        # 搜索高度必须高于圆环可完整入画的临界值
        self.assertGreater(cfg.search_z, 2.0)

    def test_missing_file_falls_back(self):
        cfg = MissionConfig.from_yaml("/nonexistent/mission.yaml")
        self.assertAlmostEqual(cfg.search_z, 2.4)

    def test_summary_contains_key_numbers(self):
        s = MissionConfig().summary()
        self.assertIn("search_z=2.40", s)
        self.assertIn("corridor=11", s)


class TestConfigFromYaml(unittest.TestCase):
    def test_override(self):
        try:
            import yaml  # noqa: F401
        except Exception:
            self.skipTest("PyYAML 不可用")
        text = "search_z: 2.8\ndrop_z: 2.5\nmax_targets: 2\ncorridor:\n  - [1.0, 2.0, 1.0]\n"
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as f:
            f.write(text)
            path = f.name
        try:
            cfg = MissionConfig.from_yaml(path)
            self.assertAlmostEqual(cfg.search_z, 2.8)
            self.assertAlmostEqual(cfg.drop_z, 2.5)
            self.assertEqual(cfg.max_targets, 2)
            self.assertEqual(cfg.corridor, [(1.0, 2.0, 1.0)])
            # 未覆盖的字段保持默认
            self.assertEqual(cfg.takeoff_z, 1.5)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
