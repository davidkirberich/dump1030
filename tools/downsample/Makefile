CC       ?= gcc
TARGETS   = downsample_sc16_to_cu8_fast downsample_sc16_to_sc16_fast
CFLAGS   ?= -O3 -march=native -ffast-math -fopenmp -Wall -Wextra
LDFLAGS  ?= -fopenmp
LDLIBS   ?= -lm

.PHONY: all clean

all: $(TARGETS)

%: %.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGETS)