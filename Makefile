CC = gcc
CFLAGS = -Wall -Iinclude
SRC = src/gpio.c src/HAL.c
OBJ = $(SRC:.c=.o)
LIBNAME = libgpio

all: static shared

static: $(OBJ)
	ar rcs lib$(LIBNAME).a $(OBJ)

shared: $(OBJ)
	$(CC) -shared -o lib$(LIBNAME).so $(OBJ)

clean:
	rm -f $(OBJ) lib$(LIBNAME).a lib$(LIBNAME).so
