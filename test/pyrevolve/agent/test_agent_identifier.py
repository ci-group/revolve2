import unittest

from pyrevolve.patterns.sequential_identifier import AgentIdentifier


class TestAgentIdentifier(unittest.TestCase):

    def test_increment(self):
        agentID = AgentIdentifier.Instance()
        index = agentID.current()

        agentID.increment()
        self.assertEqual(agentID.current(), index+1)

    def test_index(self):
        agentID = AgentIdentifier.Instance()
        agentID2 = AgentIdentifier.Instance()
        index = agentID.current()

        agentID.increment()

        self.assertEqual(agentID.current(), index+1)
        self.assertEqual(agentID2.current(), index+1)
