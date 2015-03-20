from distutils.extension import Extension

ext_modules = [Extension("aircraftpcefgm",
    ["aircraftpce/cython/aircraftpcefgm.pyx",
        "aircraftpce/aircraftpcefgmdynmem.c","aircraftpce/aircraftpcefgm.c",
        "aircraftpce/aircraftpcecvpdynmem.c","aircraftpce/aircraftpcecvp.c",
        "aircraftpce/cjson.c",
        "aircraftpce/aircraftpcemtxops.c"],
    include_dirs = ["aircraftpce/include"], library_dirs = ["aircraftpce"],
    libraries=["m"])]
