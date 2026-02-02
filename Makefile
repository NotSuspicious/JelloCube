# Jello cube Makefile 
# Jernej Barbic, USC
# William Zhao, USC

# Makefile
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
SRCS := $(wildcard src/*.cpp)
ifeq ($(strip $(SRCS)),)
$(error No source files found in `src` — make sure your `.cpp` files are in the `src` directory)
endif
OBJS := $(patsubst src/%.cpp,$(BINDIR)/%.o,$(SRCS))

.PHONY: all clean

all: jello createWorld

jello: $(OBJS)
	$(COMPILER) $(COMPILERFLAGS) -o $@ $(OBJS) $(LIBRARIES)

$(BINDIR):
	mkdir -p $(BINDIR)

$(BINDIR)/%.o: src/%.cpp | $(BINDIR)
	$(COMPILER) -c $(COMPILERFLAGS) -o $@ $<

createWorld: $(BINDIR)/createWorld.o
	$(COMPILER) $(COMPILERFLAGS) -o $@ $^ $(LIBRARIES)

clean:
	-rm -rf $(BINDIR) createWorld jello
