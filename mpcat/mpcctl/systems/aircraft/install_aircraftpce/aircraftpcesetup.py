from distutils.core import setup
from distutils.extension import Extension
from sys import path

from Cython.Build import cythonize

path.append('./aircraftpce/cython/')
from aircraftpcecvpsetup import ext_modules as former_ext_modules
from aircraftpcefgmsetup import ext_modules as solver_ext_modules
ext_modules = former_ext_modules + solver_ext_modules

setup(name='aircraftpce', packages=['aircraftpce'],
        ext_package='aircraftpce',
        ext_modules=cythonize(ext_modules),
        package_dir={'aircraftpce': 'aircraftpce'},
        package_data={'aircraftpce': ['mpc.pickle']},
        )

