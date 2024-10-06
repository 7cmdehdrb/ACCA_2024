from setuptools import setup
import os
from glob import glob

package_name = "localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="acca",
    maintainer_email="acca@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kalman_localization = localization.kalman_localization:main",
            "map_odom_tf_publisher = localization.map_odom_tf_publisher:main",
            "real_time_map_server = localization.real_time_map_server:main",
            "osm = localization.osm:main",
            "gps_localization = localization.gps_localization:main",
        ],
    },
)
