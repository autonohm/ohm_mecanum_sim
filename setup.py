from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['scripts/ohm_mecanum_sim_node.py'],
    packages=['ohm_mecanum_sim'],
    package_dir={'': 'src'}
)
