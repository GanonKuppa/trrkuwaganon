# -----------------------------------------------------------------------------
# CMake project wrapper Makefile ----------------------------------------------
# -----------------------------------------------------------------------------

ifeq ($(OS),Windows_NT)
	RM    := rmdir /s /q build > NUL 2>&1 & if ERRORLEVEL 1 cmd /c exit 0
	MKDIR := mkdir build > NUL 2>&1 & if ERRORLEVEL 1 cmd /c exit 0
	RUN_SILS   := ./build/sils_main.exe
	CMAKE_CMD  := cd build & cmake -G "MinGW Makefiles" ..
	CMAKE_NINJA_CMD := cd build & cmake ..  -G Ninja  -DCMAKE_MAKE_PROGRAM="C:/ninja-win/ninja"
	MAKE_CMD   := cd build & make
	NINJA_CMD  := cd build & C:/ninja-win/ninja
	UMAZE_SIM_RUN_CMD := cd tool/UMazeSim & npm run run	
	HELP :=echo\==== target list ==== & findstr -r "[a-z][^\.PHONY][a-z]:" Makefile
endif

.PHONY: sim
sim:
	@  $(UMAZE_SIM_RUN_CMD)

.PHONY: sils
sils:
	@- $(MKDIR)
	@  $(CMAKE_NINJA_CMD)
	@  $(NINJA_CMD)
	@  $(RUN_SILS)

.PHONY: sils_make
sils_make:
	@- $(MKDIR)
	@  $(CMAKE_CMD)
	@  $(MAKE_CMD)
	@  $(RUN_SILS)

.PHONY: sils_ninja
sils_ninja:
	@- $(MKDIR)
	@  $(CMAKE_NINJA_CMD)
	@  $(NINJA_CMD)
	@  $(RUN_SILS)

.PHONY: build_mot
build_mot:
	@ cd HardwareDebug & make -j all

.PHONY: clean
clean:
	@- $(RM)
#	@  cd HardwareDebug & make clean
	@  echo clean!

.PHONY: format
format:
	@- cd tool/astyle & code_format.bat

.PHONY: doxygen
doxygen:
	@- cd tool/doxygen & run_doxygen.bat

.PHONY: help
help:
	@- $(HELP)


.PHONY: test_simSearch
test_simSearch:	
	- rm -r test/simSearch/build
	- mkdir test/simSearch/build
	- cd test/simSearch/build; cmake .. -G Ninja                         
	- cd test/simSearch/build; ninja
	- test/simSearch/build/main

.PHONY: test_simShortest
test_simShortest:	
	- rm -r test/simShortest/build
	- mkdir test/simShortest/build
	- cd test/simShortest/build; cmake .. -G Ninja                         
	- cd test/simShortest/build; ninja
	- test/simShortest/build/main


.PHONY: test_traj
test_traj:
	- rm -r test/traj/build
	- mkdir test/traj/build
	- cd test/traj/build; cmake .. -G Ninja                         
	- cd test/traj/build; ninja
	- test/traj/build/main

.PHONY: test_trajSetpoint
test_trajSetpoint:
	- rm -r test/trajSetpoint/build
	- mkdir test/trajSetpoint/build
	- cd test/trajSetpoint/build; cmake .. -G Ninja                         
	- cd test/trajSetpoint/build; ninja
	- test/trajSetpoint/build/main
