from glob import glob
from setuptools import find_packages, setup

package_name = "tienkung_runtime_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["monitor/*.html", "monitor/*.css", "monitor/*.js"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="DedSecer",
    maintainer_email="dedsecer@example.com",
    description="Read-only JSONL runtime telemetry and local URDF web monitor",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "runtime_monitor = tienkung_runtime_monitor.runtime_monitor:main",
    ]},
)
