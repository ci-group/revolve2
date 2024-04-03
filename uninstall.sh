# !/bin/bash
# pip uninstall for every package which name contains 'revolve2'
packages=$(pip list | grep revolve2 | awk '{print $1}')
if [ -n "$packages" ]; then
    echo "The following packages will be uninstalled:"
    echo "$packages"
    read -p "Are you sure? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pip uninstall $packages -y
    fi
else
    echo "No packages to uninstall."
fi
