import asyncio

import pygazebo
import pygazebo.msg.joint_cmd_pb2


async def publish_loop():
    manager = await pygazebo.connect()

    publisher = await manager.advertise('/gazebo/default/unit_sphere/joint_cmd',
                                        'gazebo.msgs.JointCmd')

    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message.name = 'robot::joint_name'
    message.axis = 0
    message.force = 100.0

    while True:
        await publisher.publish(message)
        await asyncio.sleep(1.0)

loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())