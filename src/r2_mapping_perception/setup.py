from setuptools import setup

package_name = "r2_mapping_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="R2 Mapping",
    maintainer_email="jetson@yahboom.local",
    description="LaserScan filtering for R2 2D mapping.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_filter_node = r2_mapping_perception.scan_filter_node:main",
        ],
    },
)

