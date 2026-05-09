from setuptools import find_packages, setup

package_name = "px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jie",
    maintainer_email="jie@example.com",
    description="Mock bridge for fast ROS2 swarm tests without PX4 runtime dependencies.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_bridge = px4_bridge.px4_bridge_node:main",
            "mock_px4_bridge = px4_bridge.mock_px4_bridge_node:main",
        ],
    },
)
