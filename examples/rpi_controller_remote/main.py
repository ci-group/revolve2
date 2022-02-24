from revolve2.core.rpi_controller_remote import connect
import logging


async def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    async with connect("192.168.178.46", "pi", "raspberry") as conn:
        await conn.run_controller("test", 30)
        await conn.run_controller("test", 30)
        await conn.run_controller("test", 30)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
