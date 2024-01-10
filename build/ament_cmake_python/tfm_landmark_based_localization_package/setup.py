from setuptools import find_packages
from setuptools import setup

setup(
    name='tfm_landmark_based_localization_package',
    version='0.0.0',
    packages=find_packages(
        include=('tfm_landmark_based_localization_package', 'tfm_landmark_based_localization_package.*')),
)
