import sys

from setuptools import find_packages, setup

try:
    import pypandoc
    long_description = pypandoc.convert('README.md', 'rst')
except (IOError, ImportError, OSError) as e:
    sys.stderr.write('Failed to convert README.md to rst:\n  {}\n'.format(e))
    sys.stderr.flush()
    long_description = open('README.md').read()

for line in open('__version__.py'):
    if line.startswith('__version__ = '):
        version = line.strip().split()[2][1:-1]

setup(
    name='fcnd_drone_api',
    version=version,
    description="Drone API for Udacity's Flying Car Nanodegree",
    long_description=long_description,
    packages=find_packages(exclude=('tests*',)),
    url='https://github.com/udacity/FCND-Drone-API',
    author='Udacity FCND Team',
    # TODO: Add team email
    author_email='',
    install_requires=['numpy>=1.7', 'pymavlink>=2.2'
                      'utm>=0.4'],
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
