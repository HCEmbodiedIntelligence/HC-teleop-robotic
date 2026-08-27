from setuptools import find_packages, setup


setup(
    name="hc_teleop_v23_solver",
    version="0.1.0",
    description="Standalone reconstructed HC Pinocchio V2.3 IK solver",
    python_requires=">=3.10",
    package_dir={"": "src"},
    packages=find_packages("src"),
    py_modules=["controller_v2_3", "pinocchio_interface_v3", "solve_ik"],
    scripts=["script/control_v2_3_ros2.py"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/hc_teleop_v23_solver"],
        ),
        ("share/hc_teleop_v23_solver", ["package.xml"]),
    ],
    install_requires=["numpy", "PyYAML"],
)
