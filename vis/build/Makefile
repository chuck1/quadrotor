root_dir     = /home/chuck/home/git/drone/vis
src_dir      = /home/chuck/home/git/drone/vis/src
inc_dir      = /home/chuck/home/git/drone/vis/include
build_dir    = /home/chuck/home/git/drone/vis/build
inc          = -I/home/chuck/home/usr/lib/pmake/include -I/home/chuck/home/usr/lib/pmake/build/static/process/include -I/home/chuck/home/usr/lib/pmake/include -I/home/chuck/home/usr/lib/pmake/build/static/process/include -I/home/chuck/home/git/drone/math/include -I/home/chuck/home/git/drone/math/build/static/process/include -I/home/chuck/home/git/drone/drone/include -I/home/chuck/home/git/drone/drone/build/static/process/include -I$(inc_dir)
define_str   = 
binary       = /home/chuck/home/git/drone/build/vis
lib_long_str = /home/chuck/home/git/drone/math/build/static/libmath.a /home/chuck/home/git/drone/drone/build/static/libdrone.a
lib_link_str = -lboost_program_options -lglfw3 -lGL -lGLU -lGLEW -lfreetype -lpng -lrt -lpthread -lX11 -lXrandr -lXi -lXcursor -lXxf86vm -lXinerama -l:/home/chuck/home/git/drone/math/build/static/libmath.a -l:/home/chuck/home/git/drone/drone/build/static/libdrone.a
lib_dir_str  = -L/home/chuck/home/usr/lib/pmake/build/static -L/home/chuck/home/usr/lib/pmake/build/static -L/home/chuck/home/git/drone/math/build/static -L/home/chuck/home/git/drone/drone/build/static
compiler_dir = /home/chuck/home/git/pbs

depends_dir = $(build_dir)/depends
objects_dir = $(build_dir)/objects
process_dir = $(build_dir)/process

include $(compiler_dir)/makefiles/color.mk


#src    = $(shell find $(src_dir) -name '*.cpp')
#src_in = $(shell find $(src_dir) -name '*.cpp.in')

#inc_in = $(shell find $(inc_dir) -name '*.hpp.in')

#inc_processed = $(patsubst $(src_dir)/%.cpp.in, $(build_dir)/processed/include/%.hpp, $(inc_in))

#src_processed = $(patsubst $(src_dir)/%.cpp.in, $(build_dir)/processed/src/%.cpp,     $(src_in))
#obj_processed = $(patsubst $(src_dir)/%.cpp.in, $(build_dir)/objects/%.cpp.o, $(src_in))
#obj           = $(patsubst $(src_dir)/%.cpp,    $(build_dir)/objects/%.cpp.o, $(src))


CC = g++
CFLAGS = -g -std=c++0x -Wno-format-security -rdynamic

CARGS = $(define_str) $(CFLAGS) $(inc)

.PHONY: all

all: $(binary)
	@bash -c "echo -e \"$(COLOR_BLUE)$(build_dir)$(COLOR_RESET)\""


MAKEDEPEND = $(CC) -c $(CARGS) -MM $< -MF $(build_dir)/depends/$*.cpp.d -MT $(build_dir)/objects/$*.cpp.o

include $(compiler_dir)/makefiles/cpp_library.mk

$(binary): $(lib_long_str)

$(binary): $(obj)
	@bash -c "echo -e \"$(COLOR_BLUE)build $@$(COLOR_RESET)\""
	@$(CC) $(CARGS) $^ -o $@ $(lib_dir_str) $(lib_link_str) $(lib_link_str) $(lib_link_str) -lpthread


include $(compiler_dir)/makefiles/clean.mk
include $(compiler_dir)/makefiles/help.mk
include $(compiler_dir)/makefiles/depend.mk
include $(compiler_dir)/makefiles/doc.mk

