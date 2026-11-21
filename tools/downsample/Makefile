CC       ?= gcc
TARGET    = downsample_sc16_to_cu8_fast
SRC       = downsample_sc16_to_cu8_fast.c
CFLAGS   ?= -O3 -march=native -ffast-math -fopenmp -Wall -Wextra
LDFLAGS  ?= -fopenmp
LDLIBS   ?= -lm

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)