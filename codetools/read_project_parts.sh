#!/bin/sh

set -e

cd "$(dirname "$0")"

python3 -c "
import yaml

with open('../project.yml') as file:
    data = yaml.safe_load(file)

namespace = data['revolve2-namespace']
examples_dir = data['examples-dir']
tests_dir = data['tests-dir']

platform_dependent = [f'{pkg}/{namespace}' for pkg in data['platform_dependent_packages']]
platform_independent = [f'{pkg}/{namespace}' for pkg in data['platform_independent_packages']]
examples = [f'{examples_dir}/{example}' for example in data['examples']]

all_packages = platform_dependent + platform_independent + examples + [tests_dir]
print('\n'.join(all_packages))
"
