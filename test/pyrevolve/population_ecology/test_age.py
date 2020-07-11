import unittest

from pyrevolve.ecology import Age


class TestAgentIdentifier(unittest.TestCase):

    def test_increment(self):
        age = Age()
        index: int = 0
        self.assertEqual(age.generations, index)
        self.assertEqual(age.no_improvement, index)

        age.update(True)
        index += 1

        self.assertEquals(age.generations, index)
        self.assertEquals(age.no_improvement, index)

        age.update(False)
        index += 1

        self.assertEqual(age.generations, index)
        self.assertEqual(age.no_improvement, 0)

        age.update(True)
        index += 1

        self.assertEqual(age.generations, index)
        self.assertEqual(age.no_improvement, 1)
