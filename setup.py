import platform

from setuptools import find_packages, setup

readme = open('README.md').read()

setup(
    name='udacidrone',
    version='0.1.0',
    description="Drone API for Udacity's Flying Car Nanodegree",
    long_description=readme,
    packages=find_packages(exclude=('tests*',)),
    url='https://github.com/udacity/udacidrone',
    author='Udacity FCND Team',
    # TODO: Add team email
    author_email='',
    install_requires=[
        'numpy>=1.12',
        'future==0.16.0',
        'lxml==4.1.1',
        'pymavlink==2.2.8',
        'utm==0.4',
        'websockets==4.0.1',
    ] + (['uvloop==0.9.1'] if platform.system() is not 'Windows' else []),
    tests_require=['flake8', 'pytest'],
    keywords='drone api udacity flying car quadrotor',
    license='MIT License',
    classifiers=[
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3.6',
    ],
    # yapf
)
