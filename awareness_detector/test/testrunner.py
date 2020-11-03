"""Awareness Detector Test Runner"""

import rosunit


if __name__ == "__main__":
    rosunit.unitrun(
        "awareness_detector", "test_geometry", "test.test_geometry.GeometryTestSuite"
    )
    rosunit.unitrun("awareness_detector", "test_sa", "test.test_sa.SATestSuite")
    rosunit.unitrun("awareness_detector", "test_view", "test.test_view.ViewTestSuite")
