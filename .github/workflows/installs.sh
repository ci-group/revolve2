#/bin/bash
# helper script to install this repo's packages/dependencies (optionally in dev/edit mode).

set -ex

SCRIPT_DIR="$(realpath "$(dirname "$0")")"
ROOT_DIR="$(realpath "$SCRIPT_DIR/../..")"

function main() {
    cd "$ROOT_DIR"

    [[ "$OSTYPE" == "darwin"* ]] && export IS_MAC=true || export IS_MAC=false
    edit=()
    DEV=""
    while getopts "edh" arg; do
        case $arg in
            e)
                edit=(-e)
                echo "installing packages in edit mode!"
                ;;
            d)
                echo "installing dev dependencies!"
                DEV="[dev]"
                ;;
            h)
                usage
                exit
                ;;
        esac
    done

    echo -e "\nstarting installations..."

    pip install "${edit[@]}" ./serialization$DEV
    pip install "${edit[@]}" ./actor_controller$DEV
    pip install "${edit[@]}" ./simulation$DEV
    pip install "${edit[@]}" ./modular_robot$DEV
    pip install "${edit[@]}" ./rpi_controller$DEV
    pip install "${edit[@]}" ./rpi_controller_remote$DEV

    pip install "${edit[@]}" ./experimentation$DEV
    pip install "${edit[@]}" ./simulators/mujoco$DEV
    pip install "${edit[@]}" ./ci_group$DEV


    # install examples requirements
    pip install -r examples/robot_bodybrain_ea_database/requirements.txt
    pip install -r examples/robot_brain_cmaes_database/requirements.txt
    pip install -r examples/simple_ea_xor_database/requirements.txt

    # install unit test requirements
    pip install -r tests/requirements.txt

    if ! $IS_MAC; then
        sudo apt install -y libcereal-dev
    else
        brew install cereal
    fi
    #pip install "${edit[@]}" ./genotypes/cppnwin$DEV

    echo -e "\ninstall.sh complete!"
}

function usage() {
    echo -e "Helper script for installing dependencies."
    echo -e "\nUSAGE:"
    echo -e "\tinstalls.sh [-e] [-d] [-h]\n"
    echo -e "\tinstalls.sh -e    # install packages in edit mode (can be combined with -d)"
    echo -e "\tinstalls.sh -d    # install dev dependencies as well (can be combined with -e)"
    echo -e "\tinstalls.sh -h    # display this help and exit"
}

main "$@"