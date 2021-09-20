import unittest

from revolve2.abstract.sequential_identifier import SequentialIdentifier


class TestAgentIdentifier(unittest.TestCase):

    def test_increment(self):
        agentID = SequentialIdentifier()
        self.assertEqual(agentID.id(), 1)
        self.assertEqual(agentID.id(), 2)
        self.assertEqual(agentID.id(), 3)
