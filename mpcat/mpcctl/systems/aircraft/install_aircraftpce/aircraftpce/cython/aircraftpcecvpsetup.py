from distutils.core import setup
from distutils.extension import Extension

ext_modules = [Extension("aircraftpcecvp",
    ["aircraftpce/cython/aircraftpcecvp.pyx", "aircraftpce/cython/aircraftpceCcvp.pyx",
        "aircraftpce/aircraftpcecvp.c",
    "aircraftpce/aircraftpcecvpdynmem.c",
    "aircraftpce/cjson.c",  "aircraftpce/aircraftpcemtxops.c"],
    include_dirs = ["aircraftpce/include"], library_dirs = ["aircraftpce"],
    libraries=["m"]),
    Extension("aircraftpceCcvp",
    ["aircraftpce/cython/aircraftpceCcvp.pyx",
    ],
    include_dirs = ["aircraftpce/include"], library_dirs = ["aircraftpce"],
    libraries=["m"])]

