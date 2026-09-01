from setuptools import find_packages, setup


package_name = "hc_dataset"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=[
        "setuptools",
        "mcap>=1.4,<2",
        "mcap-ros2-support>=0.5,<1",
    ],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="HC Robotics",
    maintainer_email="robotics@hc.local",
    description="Isolated rosbag2/MCAP dataset recording, catalog and replay.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "dataset_node = hc_dataset.node:main",
            "hc_mcap_reprofile = hc_dataset.reprofile:main",
        ]
    },
)
