import platform

from setuptools import find_packages, setup

readme = open('README.md').read()

setup(
    name='udacidrone',
    version='0.3.5',
    description="Drone API for Udacity's Flying Car Nanodegree",
    long_description=readme,
    packages=find_packages(exclude=('tests*',)),
    url='https://github.com/udacity/udacidrone',
    author='Udacity FCND Team',
    # TODO: Add team email
    author_email='',
    install_requires=[
        'numpy>=1.21',
        'future==0.18.2',
        'lxml==4.9.2',
        'pymavlink==2.4.41',
        'utm==0.4',
        'websockets>=10.0',
        'cflib>=0.1.6',
    ] + (['uvloop==0.17.0'] if platform.system() is not 'Windows' else []),
    tests_require=['flake8', 'pytest'],
    keywords='drone api udacity flying car quadrotor',
    license='MIT License',
    classifiers=[
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3.10',
    ],
    # yapf
)
