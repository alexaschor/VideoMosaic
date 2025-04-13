MKDIR      ?= mkdir -p
MAKE       ?= make
RM         ?= rm -f

BIN        := bin
SRC        := src
LIB        := lib

INCLUDES   := $(addprefix -I,$(wildcard lib/* lib/*/include lib))
LIBS       := `pkg-config --cflags --libs opencv4` -lmpfr -lgmp
OPT        := -O2

CXX        := clang++
CXXFLAGS   := $(CPPFLAGS) $(OPT) $(LIBS) -std=c++20 -g -MMD $(INCLUDES) -Wno-unused-command-line-argument

SOURCES    := $(SRC)/main.cpp
OBJECTS    := $(SOURCES:.cpp=.o)
DEPENDS    := $(OBJECTS:.o=.d)
EXECUTABLE := $(BIN)/run

all: $(EXECUTABLE)

prof: CXXFLAGS += -pg -no-pie -fno-builtin
prof: $(EXECUTABLE)

%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


$(EXECUTABLE): $(OBJECTS)
	@$(MKDIR) $(BIN)
	$(CXX) $(CXXFLAGS) $^ -o $(EXECUTABLE)

$(BIN):
	$(MKDIR) $(BIN)

clean:
	$(RM) $(EXECUTABLE) $(OBJECTS) $(DEPENDS)

-include $(DEPENDS)

