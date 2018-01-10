import sys
import platform

from setuptools import find_packages, setup

try:
    import pypandoc
    long_description = pypandoc.convert('README.md', 'rst')
except (IOError, ImportError, OSError) as e:
    sys.stderr.write('Failed to convert README.md to rst:\n  {}\n'.format(e))
    sys.stderr.flush()
    long_description = open('README.md').read()

__version__ = '0.1.0'

with open('./udacidrone/__version__.py') as f:
    for line in f:
        if line.startswith('__version__ = '):
            version = line.strip().split()[2][1:-1]

setup(
    name='udacidrone',
    version=version,
    description="Drone API for Udacity's Flying Car Nanodegree",
    long_description=long_description,
    packages=find_packages(exclude=('tests*',)),
    url='https://github.com/udacity/udacidrone',
    author='Udacity FCND Team',
    # TODO: Add team email
    author_email='',
    install_requires=[
        'numpy>=1.10',
        'pymavlink>=2.2'
        'utm>=0.4',
        'websockets>=4.0.1',
    ] + (['uvloop>=0.9.1'] if platform.system() is not 'Windows' else []),
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
