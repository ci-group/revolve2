import importlib.util
import inspect
import os
from pathlib import Path
from collections import OrderedDict
import ast

def get_files_from_path(db_directory):
    """Populate the dropdown with files from the directory."""
    if os.path.exists(db_directory):
        return [f for f in os.listdir(db_directory) if os.path.isfile(os.path.join(db_directory, f))]
    else:
        print(f"There are no files in the directory {db_directory}")      


def get_functions_from_file(file_path):
    """
    Load a Python file and extract all functions defined in it, preserving the order of definition.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    
    # First, parse the source code to get function definitions in order
    with open(file_path, 'r') as f:
        source = f.read()
    
    # Parse the source code into an AST
    tree = ast.parse(source)
    
    # Extract function names in order of appearance
    function_names = []
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            function_names.append(node.name)
    
    # Remove duplicates while preserving order (in case of nested functions)
    seen = set()
    ordered_function_names = [name for name in function_names if not (name in seen or seen.add(name))]
    
    # Now load the module to get the actual function objects
    module_name = os.path.splitext(os.path.basename(file_path))[0]
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    
    # Create OrderedDict with functions in the original order
    functions = OrderedDict()
    for name in ordered_function_names:
        if hasattr(module, name):
            func = getattr(module, name)
            if inspect.isfunction(func) and func.__module__ == module_name:
                functions[name] = func
    
    # Add any remaining functions that might have been missed
    for name, obj in inspect.getmembers(module):
        if (inspect.isfunction(obj) and 
            obj.__module__ == module_name and 
            name not in functions):
            functions[name] = obj
    
    return functions

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
            module.__all__.remove('pareto_frontier')  # Remove specific function if needed
            module.__all__.remove('multiple_with_replacement')  # Remove specific function if needed
        return module.__all__
    else:
        raise AttributeError(f"__all__ not found in {init_file}")
    
def get_config_parameters_from_file(file_path):
    """Dynamically load variables from a config file as a dictionary with preserved order."""
    if not os.path.exists(file_path):
        with open(file_path, "w") as f:
            f.write("# Default config file\n")
    
    # Read the file to get the order of variables
    with open(file_path, "r") as f:
        lines = f.readlines()
    
    # Extract variable names in order of appearance
    ordered_keys = []
    for line in lines:
        line = line.strip()
        if line and not line.startswith("#"):
            # Extract the variable name before the equals sign
            if "=" in line:
                key = line.split("=")[0].strip()
                ordered_keys.append(key)
    
    # Now load the module to get the actual values
    spec = importlib.util.spec_from_file_location("config", file_path)
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    
    # Create OrderedDict with keys in the original order
    result = OrderedDict()
    for key in ordered_keys:
        if hasattr(config, key):
            result[key] = getattr(config, key)
    
    # Add any remaining keys that might have been missed
    for key in dir(config):
        if not key.startswith("__") and key not in result:
            result[key] = getattr(config, key)
    
    return result

def save_config_parameters(file_path, values):
    """Save the modified values back to a config file."""
    with open(file_path, "w") as f:
        for key, value in values.items():
            if isinstance(value, str):
                f.write(f'{key} = "{value}"\n')
            else:
                f.write(f"{key} = {value}\n")
