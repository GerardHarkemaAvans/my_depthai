  
import setuptools
from setuptools import find_packages

long_description = "https://docs.roboflow.com/inference/luxonis-oak"

setuptools.setup(
    name="roboflowoak",  # Replace with your own username
    version="0.0.8",
    author="Roboflow",
    license='GPLv3+',
    author_email="jacob@roboflow.com",
    description="python client for deploying Roboflow models to OAK devices",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://www.roboflow.com",
    install_requires=[
        "requests==2.26.0"
    ],
    packages=find_packages(include=('roboflowoak',)),
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)