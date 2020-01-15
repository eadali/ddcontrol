# -*- coding: utf-8 -*-
"""
Created on Tue Jan 14 15:05:54 2020

@author: ERADALI
"""
from distutils.core import setup
setup(
  name = 'ddcontrol',
  packages = ['ddcontrol'],
  version = '0.1',
  license='MIT',
  description = 'Control Theory for humans.',
  long_description='homepage..: https://github.com/eadali/ddcontrol',
  author = 'Erkan ADALI',
  author_email = 'erkanadali91@gmail.com',
  url = 'https://github.com/eadali/ddcontrol',
  download_url = 'https://github.com/eadali/ddcontrol/archive/v0.1.tar.gz',
  keywords = ['control', 'pid', 'tune', 'real time',],
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