from setuptools import find_packages, setup

package_name = "test_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="acca",
    maintainer_email="acca@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_pub = test_pkg.test_pub:main",
            "test_sub = test_pkg.test_sub:main",
            "feedback_odom_converter = test_pkg.feedback_odom_converter:main",
            "imu_test = test_pkg.imu_test:main",
            "kalman = test_pkg.kalman:main",
            "wheel_odomety = test_pkg.wheel_odomety:main",
            "test = test_pkg.test:main",
            "velodyne_points2 = test_pkg.velodyne_points2:main",
            "velodyne_tf = test_pkg.velodyne_tf:main",
        ],
    },
)
