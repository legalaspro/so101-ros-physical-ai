from setuptools import find_packages, setup
from glob import glob

package_name = "so101_inference"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dmitri Manajev",
    maintainer_email="dmitri@manajev.com",
    description="SO-101 policy inference nodes (LeRobot) for ROS 2",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "lerobot_inference_node = so101_inference.lerobot_inference_node:main",
            "async_inference_node = so101_inference.async_inference_node:main",
        ],
    },
)
