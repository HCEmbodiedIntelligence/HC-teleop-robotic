from glob import glob

from setuptools import find_packages, setup


package_name = "hc_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/static", glob("static/*")),
        (f"share/{package_name}/frontend", glob("frontend/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="HC Robotics",
    maintainer_email="robotics@hc.local",
    description="Read-only ROS telemetry and safety-service web dashboard.",
    license="Proprietary",
    entry_points={"console_scripts": ["dashboard = hc_dashboard.server:main"]},
)
