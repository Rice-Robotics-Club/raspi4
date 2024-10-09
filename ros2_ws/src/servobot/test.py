import os
from glob import glob

package_name = "servobot"

launch_files = glob(os.path.join("launch", "*.launch.yaml"))
executables = glob(os.path.join(package_name, "*_node.py"))

def console_script(filename: str) -> str:
    name = filename[len(package_name)+1:len(filename)-3]
    return f"{name} = {package_name}.{name}:main"

print(executables)

print(list(map(console_script, executables)))
