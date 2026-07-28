from glob import glob
from setuptools import find_packages, setup


package_name = "cbf_rover_ros"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="CBF-Rover maintainers",
    maintainer_email="noreply@example.com",
    description="Lifecycle ROS 2 bridge for external CBF-Rover methods.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lifecycle_bridge = cbf_rover_ros.lifecycle_bridge:main",
        ],
    },
)
