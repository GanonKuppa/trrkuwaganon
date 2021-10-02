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
	@  cd HardwareDebug & make clean
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
