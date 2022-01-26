import argparse
import asyncio
import cProfile

import optimize

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("out", help="Output file")
    args = parser.parse_args()
    cProfile.run("asyncio.run(optimize.main())", args.out)
