WITH_RTLSDR ?= 1
WITH_PLUTO ?= 0

CXXFLAGS ?= -O2 -g -Wall -Wextra -std=gnu++17
CPPFLAGS += -DWITH_RTLSDR=$(WITH_RTLSDR) -DWITH_PLUTO=$(WITH_PLUTO)
LDLIBS += -lpthread -lm

ifeq ($(WITH_RTLSDR),1)
CPPFLAGS += $(shell pkg-config --cflags librtlsdr)
LDLIBS += $(shell pkg-config --libs librtlsdr)
endif

ifeq ($(WITH_PLUTO),1)
CPPFLAGS += $(shell pkg-config --cflags libiio)
LDLIBS += $(shell pkg-config --libs libiio)
endif

CXX ?= g++
PROGNAME=dump1030

all: dump1030

%.o: %.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

dump1030: dump1030.o
	$(CXX) -o dump1030 dump1030.o $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o dump1030
