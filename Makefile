CC = gcc
CFLAGS = -Wall -Iinclude -fPIC
SRC = src/gpio.c src/HAL.c
OBJ = $(SRC:.c=.o)
LIBNAME = gpio

all: static shared

static: $(OBJ)
	ar rcs lib$(LIBNAME).a $(OBJ)

shared: $(OBJ)
	$(CC) -shared -o lib$(LIBNAME).so $(OBJ)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) lib$(LIBNAME).a lib$(LIBNAME).so
