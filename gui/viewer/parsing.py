import importlib.util
import inspect
import os
from pathlib import Path


def get_files_from_path(db_directory):
    """Populate the dropdown with files from the directory."""
    if os.path.exists(db_directory):
        return [f for f in os.listdir(db_directory) if os.path.isfile(os.path.join(db_directory, f))]
    else:
        print(f"There are no files in the directory {db_directory}")      


def get_functions_from_file(file_path):
    """
    Load a Python file and extract all functions defined in it.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    
    module_name = os.path.splitext(os.path.basename(file_path))[0]
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    # Extract all functions
    functions = {
        name: func
        for name, func in inspect.getmembers(module, inspect.isfunction)
    }
    return functions

def get_function_names_from_init(folder_path):
    """
    Import the __init__.py file from the given folder and extract the function names listed in the __all__ variable.
    """

    init_file = os.path.join(folder_path, "__init__.py")
    if not os.path.exists(init_file):
        raise FileNotFoundError(f"__init__.py not found in folder: {folder_path}")
    
    # Dynamically load the __init__.py file
    module_name = os.path.basename(folder_path)  # Use folder name as module name
    spec = importlib.util.spec_from_file_location(module_name, init_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    # Extract function names from __all__ list in __init__.py
    if hasattr(module, "__all__"):
        module.__all__.remove('multiple_unique')
        return module.__all__
    else:
        raise AttributeError(f"__all__ not found in {init_file}")
    
def get_selection_names_from_init():
    """
    Import the __init__.py file from the given folder and extract the function names listed in the __all__ variable.
    """
    # Get the script's directory
    script_path = Path(__file__).resolve()
    
    # Dynamically find the root of your project (assuming 'revolve2' is the project root)
    for parent in script_path.parents:
        if parent.name == "revolve2":
            project_root = parent
            break
    else:
        raise FileNotFoundError("Could not determine the project root (revolve2).")

    # Define the correct selection path dynamically
    selection_path = project_root / "experimentation" / "revolve2" / "experimentation" / "optimization" / "ea" / "selection"
    
    # Ensure selection_path exists
    if not selection_path.exists():
        raise FileNotFoundError(f"Selection path does not exist: {selection_path}")

    init_file = selection_path / "__init__.py"
    
    if not init_file.exists():
        raise FileNotFoundError(f"__init__.py not found in folder: {selection_path}")
    
    # Dynamically load the __init__.py file
    module_name = selection_path.name  # Use folder name as module name
    spec = importlib.util.spec_from_file_location(module_name, init_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    # Extract function names from __all__ list in __init__.py
    if hasattr(module, "__all__"):
        if 'multiple_unique' in module.__all__:
            module.__all__.remove('multiple_unique')  # Remove specific function if needed
        return module.__all__
    else:
        raise AttributeError(f"__all__ not found in {init_file}")

def get_config_parameters_from_file(file_path):
    """Dynamically load variables from a config file as a dictionary."""
    if not os.path.exists(file_path):
        with open(file_path, "w") as f:
            f.write("# Default config file\n")
    spec = importlib.util.spec_from_file_location("config", file_path)
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    return {key: getattr(config, key) for key in dir(config) if (not key.startswith("__"))}

def save_config_parameters(file_path, values):
    """Save the modified values back to a config file."""
    with open(file_path, "w") as f:
        for key, value in values.items():
            if isinstance(value, str):
                f.write(f'{key} = "{value}"\n')
            else:
                f.write(f"{key} = {value}\n")
