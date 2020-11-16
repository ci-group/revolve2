from revolve.robot.body.robogen_body import RobogenBody


def measure_robogen_body(robot_body: RobogenBody, encoding):
    expression_efficiency = len(robot_body.modules) / len(encoding)

    # for branch in robot_body.modules[0].children:

    # print("Expression efficiency: ", )