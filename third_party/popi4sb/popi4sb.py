import os
PATH = os.getcwd()

from simulators.run_simulation import run

def get_bool(prompt):
    while True:
        try:
           return {"true": True, "false": False}[input(prompt).lower()]
        except KeyError:
           print("Invalid input, please enter True or False!")


if __name__ == "__main__":
    print('\n\nWelcome to Population-based Parameter Identification for Systems Biology (POPI4SB)!\n')
    print(' ******************************************************\n',
          '* ___       _      ___    __              ___     __ *\n',
          '*||  \    // \    ||  \  || |  ||   ||   / /    ||  \*\n',
          '*||   |  ||   ||  ||   |  \/   ||   ||  | /     ||  /*\n',
          '*||__/   ||   ||  ||__/   __   ||_  ||   \_\    || | *\n',
          '*||      ||   ||  ||     || |     | ||     \ \  ||  \*\n',
          '*||        \_//   ||     ||_|     |_||   __/ /  ||__/*\n',
          '******************************************************')
    print('© E. Weglarz-Tomczak & J. Tomczak\n\nThis program allows to identify parameters of a model in PySCeS.\n')

    print('------------------\n')

    print(f'Current PATH: {PATH}\n'
          f'This will be used as the default path.\n'
          f'If you prefer to provide other paths, '
          f'please specify them manually.\n')

    dir_results = input(f'Please provide a folder name where the results should be saved (default: {PATH}/results/): ') or os.path.join(PATH, 'results')

    dir_method = input(f'Please provide a folder name where files for methods (.json) are stored (default: {PATH}): ') or PATH

    json_methods = [item for item in input('Please provide a list of names of files (.json) for methods (separate '
                                          'file names with a space): ').split()]
    assert len(json_methods) > 0, 'Please provide at least one method to run.'

    dir_model = input(f'Please provide a folder name where files for models (.json) are stored (default: {PATH}): ') or PATH

    json_models = [item for item in input('Please provide a list of names of files for models (.json, separate '
                                          'file names with a space): ').split()]
    assert len(json_models) > 0, 'Please provide at least one model (PySCeS).'

    dir_solver = input(f'Please provide a folder name where a file of PySCeS solver is stored (default: {PATH}): ') or PATH
    json_solver = input('Please provide a name of a file where info about the solver is stored (default: solver.json): ') or 'solver.json'

    repeats = int(input('Please provide how many times experiments should be repeated (default 1): ') or 1)
    assert repeats > 0, 'The number of repetitions cannot be lower than 0!'

    real_data = get_bool('Do you use real data? (True or False): ')

    if real_data:
        dir_data = input(f'Please provide a folder name where real data is stored (default: {PATH}): ') or PATH
        file_data = input('Please provide a name of a file where real data is stored (.npy): ')
    else:
        dir_data = None
        file_data = None

    exp_name = input('Please provide a name for files (default: `exp_`): ') or 'exp_'

    for j_model in json_models:
        for j_method in json_methods:
            for i in range(repeats):
                run(dir_method, j_method, dir_model, j_model, dir_results, dir_solver, json_solver, dir_data, file_data,
                    exp_sign=exp_name + str(i) + '_')