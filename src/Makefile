CC = g++
FLAGS = -g -std=c++17 -Wno-deprecated

INCLUDE = -I../
SOURCES = *.h *.cpp

EXENAME = shaded_renderer

all: $(SOURCES)
	$(CC) $(FLAGS) -o $(EXENAME) $(INCLUDE) $(SOURCES)

clean:
	rm -f *.o $(EXENAME)

.PHONY: all clean

