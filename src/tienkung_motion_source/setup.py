from setuptools import setup

package_name = "tienkung_motion_source"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DedSecer",
    maintainer_email="dedsecer@example.com",
    description="Python motion reference publisher for the C++ Tienkung runtime.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motion_source_node = tienkung_motion_source.motion_source_node:main",
        ],
    },
)