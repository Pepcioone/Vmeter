#Smart Makefile
VERSION = "Pepcio v1.3 (2018-12-15)"

DST = wynik
LDFLAGS = -lncurses
FLAGS =

#make auto-variable
_DIRSRC=$(shell [ -d src ] && echo src || echo ;)
DIRSRC = $(_DIRSRC)

_CXX=$(shell [ "$$(find ./$(DIRSRC) -name '*.cpp')" != "" ] && echo g++ || echo gcc;)
CXX = $(_CXX)
ifeq ($(CXX),g++)
	EXT = *.cpp
else
	EXT = *.c
endif
ifeq ($(DIRSRC),)
	SRC = $(wildcard $(EXT))
else
	SRC = $(wildcard $(DIRSRC)/$(EXT))
endif

#make scripts
wynik:
	$(CXX) -o $@ $(SRC) $(LDFLAGS) $(FLAGS)
	@echo -------------------------------------------------
	@echo $$(stat -c "[%y] %s %n" $(DST)*)

all: clean wynik
debug: clean debugset wynik
debugset:
	$(eval FLAGS = -g)
clean:
	@rm -f $(DST)

.PHONY: all clean debug debugset
