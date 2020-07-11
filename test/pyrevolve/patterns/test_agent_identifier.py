import unittest

from pyrevolve.patterns.sequential_identifier import SequentialIdentifier


class TestAgentIdentifier(unittest.TestCase):

    def test_increment(self):
        agentID = SequentialIdentifier()
        self.assertEqual(agentID.current(), 0)
        self.assertEqual(agentID.increment(), 1)
        self.assertEqual(agentID.current(), 1)
