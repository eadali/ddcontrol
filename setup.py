# -*- coding: utf-8 -*-
"""
Created on Tue Jan 14 15:05:54 2020

@author: ERADALI
"""

from distutils.core import setup
setup(
  name = 'ddcontrol',
  packages = ['ddcontrol'],
  version = '0.6',
  license='MIT',
  description = 'Control Theory for humans.',
  long_description='The design of the PID controller is based entirely on experimental data collected from the plant.',
  author = 'Erkan ADALI',
  author_email = 'erkanadali91@gmail.com',
  url = 'https://github.com/eadali/ddcontrol',
  project_urls={'Documentation': 'https://ddcontrol.readthedocs.io/',
                'Source Code': 'https://github.com/eadali/ddcontrol',},
  download_url = 'https://github.com/eadali/ddcontrol/archive/v0.5.tar.gz',
  keywords = ['data', 'driven', 'control', 'theory' 'pid', 'tune', 'real', 'time', 'optimize',],
  install_requires=['numpy', 'scipy',],
  classifiers=[
    'Development Status :: 3 - Alpha',
    'Intended Audience :: Developers',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3.4',
    'Programming Language :: Python :: 3.5',
    'Programming Language :: Python :: 3.6',
  ],
)