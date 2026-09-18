from setuptools import setup

package_name = "r2_mapping_tools"

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
    description="Health checks and map tools for R2 mapping.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "health_check = r2_mapping_tools.health_check:main",
            "map_eval = r2_mapping_tools.map_eval:main",
            "odom_calibrate = r2_mapping_tools.odom_calibrate:main",
            "ps2_teleop = r2_mapping_tools.ps2_teleop:main",
        ],
    },
)
