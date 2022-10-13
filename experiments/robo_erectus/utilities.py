from genericpath import isfile
import os
from typing_extensions import LiteralString

DATABASE_PATH = "./database"
ANALYSIS_DIR_NAME = "analysis"
LASTEST_RUN_FILENAME = "latest"


def ensure_dirs(*dir_paths):
    for dir_path in dir_paths:
        if not os.path.isdir(dir_path):
            os.mkdir(dir_path)


def find_dir(parent_path: str, target_name: str) -> str:
    target_dir = os.path.join(parent_path, target_name)
    if not os.path.isdir(target_dir):
        for dir_name in sorted(os.listdir(parent_path)):
            if dir_name.startswith(target_name):
                return dir_name
        return None
    else:
        return target_name


def get_latest_run() -> str:
    target = os.path.join(DATABASE_PATH, LASTEST_RUN_FILENAME)
    if os.path.isfile(target):
        with open(target, "r") as file:
            full_run_name = file.read().rstrip()
            return full_run_name
    else:
        return None


def set_latest_run(full_run_name: str):
    target = os.path.join(DATABASE_PATH, LASTEST_RUN_FILENAME)
    with open(target, "w") as file:
        file.write(full_run_name)
