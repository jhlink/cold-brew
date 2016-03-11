# Copyright (c) 2016 FirstBuild
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# common headers for all modes
override HEADERS += coldbrew.h

# additional header files for `debug` mode
debug: override HEADERS += test/arduino.h
debug: override HEADERS += test/serial.h

# common compiler flags for all modes
override CFLAGS += -ansi
override CFLAGS += -pedantic
override CFLAGS += -Iinclude
override CFLAGS += -Wall
override CFLAGS += -Wextra
override CFLAGS += -Werror

# additional compiler flags for `debug` mode
debug: override CFLAGS += -g
debug: override CFLAGS += -O0
debug: override CFLAGS += -fprofile-arcs
debug: override CFLAGS += -ftest-coverage

# additional linker flags for `debug` mode
debug: override LDFLAGS += -fprofile-arcs

# common objects for all modes
override OBJECTS += main.o

.PHONY: all
all: test

.PHONY: clean
clean:
	rm -rf build

.PHONY: debug
debug: build/debug/coldbrew

.PHONY: test
test: debug
	./build/debug/coldbrew
	coveralls -b . --gcov-options '\-lp' --dryrun

.PHONY: coveralls
coveralls: test
	coveralls -b . --gcov-options '\-lp'
	
build/debug/coldbrew: $(addprefix build/debug/, $(OBJECTS))
	$(CC) $(LDFLAGS) -o $@ $^

build/debug/main.o: test/main.cpp $(HEADERS) build/debug
	$(CC) -c $(CFLAGS) -o $@ $<

build:
	mkdir $@

build/debug: build
	mkdir $@
