CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -g -O2
LDFLAGS = -lm

COMMON_SRC = ../common/ltsp_pdu.c \
             ../common/ltsp_sequence.c \
             ../common/ltsp_min_filter.c \
             ../common/ltsp_regression.c \
             ../common/ltsp_pio_timestamp.c

TEST_SRC = test_main.c \
           test_pdu.c \
           test_sequence.c \
           test_min_filter.c \
           test_regression.c \
           test_pio_timestamp.c

TARGET = ltsp_test

all: $(TARGET)

$(TARGET): $(COMMON_SRC) $(TEST_SRC)
	$(CC) $(CFLAGS) -I../common -o $@ $^ $(LDFLAGS)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all test clean
