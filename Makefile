CFLAGS       = -I out/common -I include -fPIC -g -Wall#-pedantic -Wall -Wextra -ggdb3
LDFLAGS      = -shared -lrt

TESTCFLAGS   = -I out/common -I include -g -Wall
TESTLDFLAGS  = -Lout -lmavlink

SHELL = /bin/sh
CC    = gcc

DEBUGFLAGS   = -O0 -D _DEBUG
RELEASEFLAGS = -O2 -D NDEBUG -combine -fwhole-program

TARGET  = ./out/libmavlink.so
SOURCES = $(shell echo src/*.c)
HEADERS = $(shell echo include/*.h)
OBJECTS = $(SOURCES:.c=.o)

PREFIX = $(DESTDIR)/usr/local
BINDIR = $(PREFIX)/bin

all: $(TARGET)

clean:
	rm -fr ./src/*.o $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(CFLAGS) $(DEBUGFLAGS) -o $(TARGET) $(OBJECTS)

tests: $(TARGET)
	$(CC) $(TESTCFLAGS) -o ./out/test_drone test/test_drone.c $(TESTLDFLAGS)
	$(CC) $(TESTCFLAGS) -o ./out/test_gcs test/test_gcs.c $(TESTLDFLAGS)
