from glob import glob
from setuptools import find_packages, setup


package_name = "cbf_rover_sim"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/xacro", glob("xacro/*.xacro")),
        (
            "share/" + package_name + "/models/obstacle_box",
            glob("models/obstacle_box/*"),
        ),
        (
            "share/" + package_name + "/models/cbf_rover_ground",
            glob("models/cbf_rover_ground/model.*"),
        ),
        (
            "share/" + package_name + "/models/cbf_rover_ground/meshes",
            glob("models/cbf_rover_ground/meshes/*"),
        ),
        (
            "share/" + package_name + "/models/cbf_rover_ground/textures",
            glob("models/cbf_rover_ground/textures/*.png"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CBF-Rover maintainers",
    maintainer_email="noreply@example.com",
    description="Gazebo simulation adapter and resources for CBF-Rover.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "obstacle_spawner = cbf_rover_sim.obstacle_spawner:main",
        ],
    },
)
