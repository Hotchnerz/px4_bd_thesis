from setuptools import setup

package_name = "doggo_walker"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "python-dotenv", "bosdyn-client==4.0.1"],
    zip_safe=True,
    maintainer="radam",
    maintainer_email="radam@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "find_spot = doggo_walker.find_spot:main",
            "spot_pos_send = doggo_walker.spot_pos_send:main",
        ],
    },
)
