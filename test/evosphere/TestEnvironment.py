from revolve.evosphere.environment import Environment


class TestEnvironment(Environment):

    def __init__(self, name: str = "test"):
        super().__init__(name)


class TestEnvironmentA(TestEnvironment):

    def __init__(self):
        super().__init__("A")


class TestEnvironmentB(TestEnvironment):

    def __init__(self):
        super().__init__("B")


class TestEnvironmentC(TestEnvironment):

    def __init__(self):
        super().__init__("C")