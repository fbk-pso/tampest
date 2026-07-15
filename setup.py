#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name="tampest",
    version="0.0.1",
    description="TAMPEST",
    packages=find_packages(),
    include_package_data=True,
    license="GPL-3.0",
    python_requires=">=3.10",
    install_requires=[
        "numpy==1.26.1",
        "shapely==2.0.7",
        "trimesh==4.6.6",
        "Pillow==10.2.0",
        "pyyaml==6.0.1",
        "opencv-python-headless==4.11.0.86",
        "pycollada==0.9",
        "python-fcl==0.7.0.8",
        "alphashape==1.3.1",
        "ompl==1.7.0",
        "pysmt @ git+https://github.com/pysmt/pysmt.git",
        "up-tempest==0.1.0",
    ],
    extras_require={
        "plot": [
            "matplotlib==3.8.3",
            "scipy==1.15.2",
            "pyvista==0.48.2",
        ],
    },
)
