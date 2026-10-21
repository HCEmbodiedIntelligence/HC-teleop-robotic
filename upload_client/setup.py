from setuptools import find_packages, setup

setup(
    name="humanoid-upload-client", version="1.4.0", python_requires=">=3.10",
    packages=find_packages(), package_data={"humanoid_upload": ["schemas/*.json"]},
    install_requires=["jsonschema>=4.18,<5", "mcap>=1.1,<2"],
    entry_points={"console_scripts": ["humanoid-upload=humanoid_upload.cli:main"]},
)
