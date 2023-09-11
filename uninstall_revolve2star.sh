# pip uninstall -y for everything that looks like 'revolve2*'
pip list | grep revolve2 | awk '{print $1}' | xargs pip uninstall -y