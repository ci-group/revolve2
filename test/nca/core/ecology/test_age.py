import unittest

from nca.core.actor.age import Age


class TestAge(unittest.TestCase):

    def test_increment(self):
        age = Age()
        index: int = 0
        self.assertEqual(age.generations, index)

        for i in range(3):
            age.update()
            index += 1

            self.assertEqual(age.generations, index)
