#
# rv64-emu -- Simple 64-bit RISC-V simulator
#
# Copyright (C) 2016,2019  Leiden University, The Netherlands.
#



CXXFLAGS = -std=c++17 -Wall -Weffc++ -g -Og
LDFLAGS = -lstdc++fs

OBJECTS = \
	alu.o \
	Adder.o \
	config-file.o \
	ControlSignal.o \
	elf-file.o \
	Enums.o \
	inst-decoder.o \
	inst-formatter.o \
	InstructionBase.o \
	IType.o \
	RType.o \
	SType.o \
    DType.o \
    JType.o \
    FType.o \
    JRegType.o \
	main.o \
	memory.o \
	memory-bus.o \
	memory-control.o \
	HazardDetection.o \
	pipeline.o \
	processor.o \
	serial.o \
	stages.o \
	sys-status.o \
	testing.o

OBJECTS_FB = framebuffer.o

HEADERS = \
	alu.h \
	Adder.h \
	arch.h \
	config-file.h \
	ControlSignal.h \
	elf-file.h \
	Enums.h \
	inst-decoder.h \
	InstructionBase.h \
	IType.h \
	RType.h \
	SType.h \
    DType.h \
    JType.h \
    JRegType.h \
	memory.h \
	memory-bus.h \
	memory-control.h \
	memory-interface.h \
	mux.h \
	pipeline.h \
	processor.h \
	reg-file.h \
	serial.h \
	stages.h \
	sys-status.h \
	testing.h

HEADERS_FB = framebuffer.h


ifdef ENABLE_FRAMEBUFFER
OBJECTS += $(OBJECTS_FB)
HEADERS += $(HEADERS_FB)

# For when the SDL2 package was installed normally
CXXFLAGS += -DENABLE_FRAMEBUFFER `pkg-config --cflags sdl2`
LDFLAGS  +=`pkg-config --libs sdl2`
endif


all:    	rv64-emu

rv64-emu:	$(OBJECTS)
		$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

%.o:		%.cc $(HEADERS)
		$(CXX) $(CXXFLAGS) -c $<

clean:
		rm -f rv64-emu
		rm -f $(OBJECTS) $(OBJECTS_FB)

check:		rv64-emu
		./test_instructions.py
