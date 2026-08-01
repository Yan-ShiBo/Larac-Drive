from glob import glob
from setuptools import find_packages, setup


package_name = "cbf_rover_runtime"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CBF-Rover maintainers",
    maintainer_email="noreply@example.com",
    description="ROS 2 runtime adapters for CBF-Rover simulation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            (
                "external_command_relay = "
                "cbf_rover_runtime.external_command_relay:main"
            ),
            (
                "polynomial_tracker = "
                "cbf_rover_runtime.polynomial_tracker:main"
            ),
            (
                "scenario_visualizer = "
                "cbf_rover_runtime.scenario_visualizer:main"
            ),
        ],
    },
)
