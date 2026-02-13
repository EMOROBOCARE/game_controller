from setuptools import find_packages, setup

package_name = "led_service_mock"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EMOROBCARE",
    maintainer_email="emorobcare@example.com",
    description="Simple LED mock UI for local development.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "led_service_mock_node = led_service_mock.node:main",
        ],
    },
)
