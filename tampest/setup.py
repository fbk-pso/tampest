#!/usr/bin/env python3

from setuptools import setup, find_packages
import tampest

long_description = \
"""============================================================
    UP_SAIPEM
 ============================================================
"""

setup(
    name="tampest",
    version=tampest.__version__,
    description="Tampest",
    packages=find_packages(),
    include_package_data=True,
    license="APACHE",
    python_requires=">=3.7"
)
