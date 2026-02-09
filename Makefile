# Jello cube Makefile 
# Jernej Barbic, USC
# William Zhao, USC

# makefile
KERNEL=$(shell uname -s)

ifeq ($(KERNEL),Linux)
 LIBRARIES = -lGL -lGLU -lglut
else
ifeq ($(KERNEL),Darwin)
 LIBRARIES = -framework OpenGL -framework GLUT
endif
endif

COMPILER = g++
COMPILERFLAGS = -O2 -I src

BINDIR = bin

# Hard-coded list of sources (replace/add files as needed)
SRCS := \
 src/jello.cpp \
 src/physics.cpp \
 src/input.cpp \
 src/pic.cpp \
 src/ppm.cpp \
 src/showCube.cpp

# Map src/xxx.cpp -> bin/xxx.o
OBJS := $(patsubst src/%.cpp,$(BINDIR)/%.o,$(SRCS))

.PHONY: all clean

all: jello createWorld

jello: $(OBJS)
	$(COMPILER) $(COMPILERFLAGS) -o $@ $(OBJS) $(LIBRARIES)

# Generic rule: compile src/%.cpp to bin/%.o (creates directories as needed)
$(BINDIR)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(COMPILER) -c $(COMPILERFLAGS) -o $@ $<

# createWorld: assume source is src/createWorld.cpp (object -> bin/createWorld.o)
CREATE_SRC := createWorld.cpp
CREATE_OBJ := $(patsubst src/%.cpp,$(BINDIR)/%.o,$(CREATE_SRC))

createWorld: $(CREATE_OBJ)
	$(COMPILER) $(COMPILERFLAGS) -o $@ $^ $(LIBRARIES)

clean:
	-rm -rf $(BINDIR) createWorld jello