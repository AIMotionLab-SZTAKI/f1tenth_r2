from setuptools import setup, find_packages

package_name = 'aimotion_fleet_manager'

setup(name=package_name,
      version='1.0.0',
      packages= find_packages(),
      install_requires=[
        'numpy',
        'PyQt5',
        'pyyaml',
        'numpy',
        'pyqtgraph',
        'scipy',
        'paramiko',
        ],
      entry_points = {
        'console_scripts': ['fleet_manager=aimotion_fleet_manager.main_ui:main'],
    },
    include_package_data=True)