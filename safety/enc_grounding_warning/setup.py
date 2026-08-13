from setuptools import find_packages, setup

import os
from glob import glob


package_name = "enc_grounding_warning"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml") + glob("config/*.xml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/docs", glob("../docs/*.md")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="USV Developer",
    maintainer_email="developer@usv.com",
    description="Simulation grounding warning nodes based on a depth grid matrix.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "depth_provider_node = enc_grounding_warning.depth_provider_node:main",
            "ukc_estimator_node = enc_grounding_warning.ukc_estimator_node:main",
            "grounding_warning_node = enc_grounding_warning.grounding_warning_node:main",
            "route_depth_publisher_node = enc_grounding_warning.route_depth_publisher_node:main",
            "generate_depth_grid = enc_grounding_warning.scripts.generate_depth_grid:main",
            "publish_test_plan = enc_grounding_warning.scripts.publish_test_plan:main",
            "integration_test = enc_grounding_warning.scripts.integration_test:main",
        ],
    },
)
