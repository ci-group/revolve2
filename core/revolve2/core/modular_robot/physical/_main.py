from brain.brain import Brain  # type: ignore # Will be added when brain is exported


async def async_main() -> None:
    brain = Brain()
    print("Hello world!")


def main() -> None:
    import asyncio

    asyncio.run(async_main())


if __name__ == "__main__":
    main()
