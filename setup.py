from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/ohm_mecanum_simulator_node.py'],
    packages=['ohm_mecanum_simulator'],
    package_dir={'': 'src'}
)