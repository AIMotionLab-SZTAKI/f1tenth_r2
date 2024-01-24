from setuptools import setup, find_packages

setup(name='aimotion_f1tenth_r2',
      version='1.0.0',
      packages=find_packages(),
      install_requires=[
        'numpy',
        'PyQt5',
        'pyyaml',
        'numpy',
        'pyqtgraph',
        'scipy',
        'paramiko',
        ],
      entry_points={
        'console_scripts': [
            'fleet_manager = main_ui:main'
        ]
    })