include files/vars.make


CC = g++
export CC

.PHONY: all2

all2: Makefile $(make_files) $(projects)

all3: Makefile $(make_files)
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile  --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile  --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile  --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile  --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile  --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile  --no-print-directory

help:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile help --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile help --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile help --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile help --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile help --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile help --no-print-directory

clean:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile clean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile clean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile clean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile clean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile clean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile clean --no-print-directory

clean_pre:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile clean_pre --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile clean_pre --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile clean_pre --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile clean_pre --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile clean_pre --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile clean_pre --no-print-directory

dependclean:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile dependclean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile dependclean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile dependclean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile dependclean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile dependclean --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile dependclean --no-print-directory

depend:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile depend --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile depend --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile depend --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile depend --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile depend --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile depend --no-print-directory

.PHONY: glut
.PHONY: vis
.PHONY: drone
.PHONY: math
.PHONY: sim
.PHONY: interactive
.PHONY: boost_program_options
.PHONY: glfw
.PHONY: glut_PRE
.PHONY: vis_PRE
.PHONY: drone_PRE
.PHONY: math_PRE
.PHONY: sim_PRE
.PHONY: interactive_PRE
.PHONY: boost_program_options_PRE
.PHONY: glfw_PRE

glut_PRE: boost_program_options_PRE glut_PRE glfw_PRE math_PRE drone_PRE
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile --no-print-directory

vis_PRE: boost_program_options_PRE glfw_PRE math_PRE drone_PRE
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile --no-print-directory

drone_PRE: math math_PRE
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile --no-print-directory

math_PRE:  
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile --no-print-directory

sim_PRE: boost_program_options_PRE math_PRE drone_PRE
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile --no-print-directory

interactive_PRE: boost_program_options_PRE glfw_PRE glut_PRE math_PRE drone_PRE
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile --no-print-directory

glut: glut_PRE 

vis: vis_PRE 

drone: drone_PRE 

math: math_PRE 

sim: sim_PRE 

interactive: interactive_PRE 



doc:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile doc --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile doc --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile doc --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile doc --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile doc --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile doc --no-print-directory

header_dep:
	@$(MAKE) -f /home/chuck/home/git/drone/math/build/static/Makefile precompiler --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/drone/build/static/Makefile precompiler --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/sim/build/Makefile precompiler --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/vis/build/Makefile precompiler --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/glut/build/Makefile precompiler --no-print-directory
	@$(MAKE) -f /home/chuck/home/git/drone/interactive/build/Makefile precompiler --no-print-directory

doc_all:
	@doxygen Doxyfile

Makefile: $(config_file_str) $(compiler_file)
	@pbs gen ..

