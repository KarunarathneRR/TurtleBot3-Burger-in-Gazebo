from setuptools import setup
package_name = 'tb3_dqn_nav'
setup(
  name=package_name,
  version='0.0.1',
  packages=[package_name],
  install_requires=['setuptools','torch'],
  entry_points={
    'console_scripts': [
      'dqn_train = tb3_dqn_nav.dqn_train:main',
      'dqn_run   = tb3_dqn_nav.dqn_run:main',
    ],
  },
)
