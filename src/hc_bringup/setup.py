from setuptools import find_packages, setup


package_name = "hc_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/teleop.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="HC Teleop Team",
    maintainer_email="czy33114@gmail.com",
    description="Profile validation, workspace doctor, and native ROS 2 bringup.",
    license="Proprietary",
    entry_points={"console_scripts": ["hcctl = hc_bringup.cli:main"]},
)
