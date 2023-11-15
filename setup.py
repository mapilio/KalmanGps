import io
import os
import re

import setuptools


def get_long_desc():
    base_dir = os.path.abspath(os.path.dirname(__file__))
    with io.open(os.path.join(base_dir, "README.md"), encoding = "utf-8") as f:
        return f.read()

def get_requirements():
    with open("requirements.txt", encoding = "utf-8") as f:
        return f.read().splitlines()

def get_version():
    cwd = os.path.abspath(os.path.dirname(__file__))
    current_version = os.path.join(cwd, "version.py")
    with io.open(current_version, encoding = "utf-8") as f:
        return re.search(r'^__version__ = [\'"]([^\'"]*)[\'"]', f.read(), re.M).group(1)

setuptools.setup(
    name = "KalmanGps",
    version = get_version(),
    author = "Mapilio",
    description = "Kalman Filter for GPS Metadata Correction",
    long_description = get_long_desc(),
    long_description_content_type='text/markdown',
    url = "https://github.com/mapilio/KalmanGps",
    license='MIT License',
    python_requires='>=3.6',
    install_requires = get_requirements(),
    entry_points = {
        "console_scripts": [
            "KalmanGps=KalmanGps:__init__"
        ]
    }
)
