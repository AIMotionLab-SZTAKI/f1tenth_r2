from setuptools import setup, find_packages

setup(name='crazymocap',
      version='1.0.0',
      packages=find_packages(),
      install_requires=[
        'pyusb',
        'motioncapture',
        'numpy',
        'PyQt5',
        'pyyaml',
        'numpy',
        'pyqtgraph'
        ]
      )
