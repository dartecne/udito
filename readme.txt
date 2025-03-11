# README
# Problemas and solutions

10/03/2025 - ERROR: "No module named 'em'" al hacer colcon build --packages-select tutorial_interfaces
Solution: https://robotics.stackexchange.com/questions/79663/python-module-empy-missing-tutorials
pip uninstall em
pip install empy

O tambien:
pip uninstall em
pip install empy==3.3.4

- ERROR: "No module named catkin_pkg"
pip install catkin_pkg lark