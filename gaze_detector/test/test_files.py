"""Files Tests"""

import unittest
from unittest.mock import patch, MagicMock
from pathlib import Path
from gaze_detector import files


class FilesTest(unittest.TestCase):
    """Files Test"""

    @patch("pathlib.Path.exists", MagicMock(return_value=False))
    def test_get_rotating_data_path__no_dir_exists__rotation_index_0(self):
        result = str(files.get_rotating_data_path(""))

        self.assertIn("0", result)

    @patch("pathlib.Path.exists", MagicMock(side_effect=[True, True, False]))
    def test_get_rotating_data_path__two_dir_exists__rotation_index_2(self):
        result = str(files.get_rotating_data_path(""))

        self.assertIn("2", result)


class FilesTestSuite(unittest.TestSuite):
    """Files Test Suite"""

    def __init__(self):
        super(FilesTestSuite, self).__init__()
        self.addTest(FilesTest())
