"""Gaze Detector Test Runner"""

import rosunit


if __name__ == "__main__":
    rosunit.unitrun(
        "gaze_detector",
        "test_geometry",
        "test.test_geometry.GeometryTestSuite",
    )
    rosunit.unitrun(
        "gaze_detector",
        "test_files",
        "test.test_files.FilesTestSuite",
    )
    rosunit.unitrun(
        "gaze_detector",
        "test_pupil",
        "test.test_pupil.PupilTestSuite",
    )
