# Copyright 2022 University of Toronto
#  
# Permission is hereby granted, to use this software and associated 
# documentation files (the "Software") in course work at the University 
# of Toronto, or for personal use. Other uses are prohibited, in 
# particular the distribution of the Software either publicly or to third 
# parties.
# 
# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#


########################################################################
## Makefile for basic_application
########################################################################

# Usage: make {all | clean} CONF={debug|release} VERBOSE={0|1}
# Default is the "all" target, debug build, and verbose output

VERBOSE ?= 1
ifeq ($(VERBOSE),1)
	ECHO := 
else
	ECHO := @
endif

#Default build type
CONF ?= debug

# the compiler
CXX = g++

# target EXE
TARGET_DIR = .
TARGET = basic_application

# the version of GTK being used
GTK_VERSION_NUM = 3.0

# the base directory of EZGL
EZGL_DIR = ezgl

# get the source and header files for the application and from EZGL
SRCS = $(wildcard ./*.cpp $(EZGL_DIR)/*.cpp)
HDRS = $(wildcard ./*.h $(EZGL_DIR)/*.hpp)

# the GTK include directories. Runs "pkg-config --cflags gtk+-3.0" to get the include directories.
GTK_INCLUDE_DIRS := $(shell pkg-config --cflags gtk+-$(GTK_VERSION_NUM) x11)

# the GTK libraries. Runs "pkg-config --libs gtk+-3.0" to get the libraries to include for GTK.
GTK_LIBS := $(shell pkg-config --libs gtk+-$(GTK_VERSION_NUM) x11)

# set the include directories
INC_DIRS = . $(EZGL_DIR) $(EZGL_DIR)/..

# compiler flags
CXX_FLAGS = -g -Wall -std=c++14

ifeq (release, $(CONF))
	CXX_FLAGS += -O3
else ifeq (debug, $(CONF))
# Don't change anything
else
    $(error Invalid value for CONF: '$(CONF)', must be 'release' or 'debug'. Try 'make help' for usage)
endif

# 'make all' target
all: $(TARGET_DIR)/$(TARGET)

# create the exe
$(TARGET_DIR)/$(TARGET) : Makefile $(SRCS)
	$(ECHO) $(CXX) $(CXX_FLAGS) $(foreach D,$(INC_DIRS),-I$D) $(GTK_INCLUDE_DIRS) $(SRCS) $(GTK_LIBS) -o $(TARGET_DIR)/$(TARGET)

# clean the EXE 
clean:
	$(ECHO) rm -f $(TARGET_DIR)/$(TARGET)

help:
	@echo "Makefile for ezgl example program"
	@echo ""
	@echo "Usage: "
	@echo '    > make -j4'
	@echo "        Call the default make target (all)."
	@echo "        This builds the project executable: '$(TARGET)'."
	@echo "        Use -j4 option to do parallel builds."
	@echo "    > make clean"
	@echo "        Removes any generated files including exectuables "
	@echo "        and object files."
	@echo "    > make help"
	@echo "        Prints this help message."
	@echo ""
	@echo ""
	@echo "Configuration Variables: "
	@echo "    CONF={release | debug}"
	@echo "        Controls whether the build performs compiler optimizations"
	@echo "        to improve performance. Currently set to '$(CONF)'."
	@echo ""
	@echo "        With CONF=release compiler optimization is enabled."
	@echo ""
	@echo "        With CONF=debug compiler optimization is disabled to improve"
	@echo "        interactive debugging."


.PHONY: all clean help
	
