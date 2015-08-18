import pbs.classes.Executable

e = pbs.classes.Executable.Executable("sim", self)

"""
e.require("galaxy_std")
e.require("galaxy_net")
e.require("galaxy_console")
e.require("nebula_fnd0")
e.require("nebula_python","dynamic")
e.require("nebula_fnd1")
e.require("physx")
e.require("boost_filesystem")
e.require("boost_system")
e.require("boost_serialization")
e.require("boost_python")
e.require("boost_thread")
e.require("python27")
"""

e.require("boost_program_options")
e.require("math")
e.require("drone")

e.make()


