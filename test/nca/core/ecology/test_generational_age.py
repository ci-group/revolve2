import unittest

from revolve2.nca.core.actor.age import GenerationalAge


class TestGenerationalAge(unittest.TestCase):

    def test_increment(self):
        age = GenerationalAge()
        index: int = 0
        self.assertEqual(age.no_improvement_count, index)

        for i in range(3):
            age.increment(True)
            index += 1

            self.assertEqual(age.no_improvement_count, index)

        age.increment(False)
        index += 1

        self.assertEqual(age.no_improvement_count, 0)
