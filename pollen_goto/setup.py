from setuptools import setup

package_name = "pollen_goto"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="remi",
    maintainer_email="remifabre1800@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goto_server = pollen_goto.goto_action_server:main",
            "goto_client_test = pollen_goto.goto_action_client:main",
        ],
    },
)
