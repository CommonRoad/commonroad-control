import unittest
from example import main



class TestExample(unittest.TestCase):
    """
    Tests that example is running
    """

    def test_example_running(self) -> None:
        main(simulate=True)

