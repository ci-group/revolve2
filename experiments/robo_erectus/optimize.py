#!/usr/bin/env python3
"""Setup and running of the optimize modular program."""

import argparse
import logging
import wandb
from random import Random

import multineat
from genotype import random as random_genotype
from optimizer import Optimizer
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.optimization import ProcessIdGen


async def main() -> None:
    """Run the optimization process."""

    parser = argparse.ArgumentParser()
    # TODO: generate name based on timestamp by default & handle loading latest one in rerun_best.py
    parser.add_argument("-n", "--experiment_name", type=str, default="default")
    parser.add_argument("--rng_seed", type=int, default=420)
    parser.add_argument("--num_initial_mutations", type=int, default=10)
    parser.add_argument("-t", "--simulation_time", type=int, default=30)
    parser.add_argument("--sampling_frequency", type=int, default=10)
    parser.add_argument("--control_frequency", type=int, default=10)
    parser.add_argument("-p", "--population_size", type=int, default=10)
    parser.add_argument("--offspring_size", type=int, default=10)
    parser.add_argument("-g", "--num_generations", type=int, default=50)
    parser.add_argument("-w", "--wandb", action="store_true")
    parser.add_argument("--wandb_os_logs", action="store_true")
    parser.add_argument("-d", "--debug", action="store_true")
    args = parser.parse_args()
    
    wandb.init(
        mode="online" if args.wandb else "disabled",
        project="robo-erectus",
        entity="ea-research",
        config=vars(args),
        settings=wandb.Settings(
            _disable_stats=not args.wandb_os_logs,
            _disable_meta=not args.wandb_os_logs,
        ),
    )
    wandb.run.name = f"{args.experiment_name}__{wandb.run.name}"

    logging.basicConfig(
        level=logging.INFO if not args.debug else logging.DEBUG,
        format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
    )

    # random number generator
    rng = Random()
    rng.seed(args.rng_seed)

    # database
    database = open_async_database_sqlite(f"./database/{args.experiment_name}")

    # process id generator
    process_id_gen = ProcessIdGen()
    process_id = process_id_gen.gen()

    # multineat innovation databases
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    initial_population = [
        random_genotype(innov_db_body, innov_db_brain, rng, args.num_initial_mutations)
        for _ in range(args.population_size)
    ]

    maybe_optimizer = await Optimizer.from_database(
        database=database,
        process_id=process_id,
        innov_db_body=innov_db_body,
        innov_db_brain=innov_db_brain,
        rng=rng,
        process_id_gen=process_id_gen,
    )
    if maybe_optimizer is not None:
        print("Initilized with existing database...")
        # TODO: if run is already finished, don't log it to wandb
        optimizer = maybe_optimizer
    else:
        print("Initialized a new experiment...")
        optimizer = await Optimizer.new(
            database=database,
            process_id=process_id,
            initial_population=initial_population,
            rng=rng,
            process_id_gen=process_id_gen,
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            simulation_time=args.simulation_time,
            sampling_frequency=args.sampling_frequency,
            control_frequency=args.control_frequency,
            num_generations=args.num_generations,
            offspring_size=args.offspring_size,
        )

    logging.info("Starting optimization process...")

    await optimizer.run()

    logging.info("Finished optimizing.")


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
