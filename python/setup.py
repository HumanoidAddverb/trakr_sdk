from setuptools import find_packages, setup
from setuptools.dist import Distribution

class BinaryDistribution(Distribution):
    """Distribution which always forces a binary package with platform name"""
    def has_ext_modules(foo):
        return True

setup(
    name="trakr_sdk",
    version="1.0.0",
    author="Addverb Technologies",
    author_email="humanoid@addverb.com",
    packages=["trakr_sdk"],
    package_data= {
        "trakr_sdk" : ["lib/amd64/*"], 
    },
    description="Trakr SDK for Communication over Socket",
    install_requires=[
        "socket", 
        "numpy>=1.19",
    ],
    distclass=BinaryDistribution
)