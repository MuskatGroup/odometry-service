"""Install runtime libraries from their canonical sources; no pip/network step during colcon."""
from os.path import relpath
from pathlib import Path

from setuptools import setup

ROOT = Path(__file__).resolve().parents[3]
PACKAGES = ["odometry_core", "odometry_geometry", "odometry_io"]

setup(
    name="odometry_python",
    version="0.1.0",
    packages=PACKAGES,
    package_dir={name: str(ROOT / "python" / name / "src" / name) for name in PACKAGES},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/odometry_python"]),
        ("share/odometry_python", ["package.xml"]),
        ("share/odometry_python/models", [relpath(p, Path(__file__).parent) for p in (ROOT / "configs/models").glob("*.yaml")]),
        ("share/odometry_python/Pathgraph", [relpath(p, Path(__file__).parent) for p in (ROOT / "dataset/Pathgraph").glob("*.json")]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Odometry team",
    maintainer_email="team@example.invalid",
    description="Transport-independent reserve odometry runtime libraries and model assets",
    license="MIT",
)
