CC = gcc
CFLAGS = -Wall -Iinclude -fPIC
SRC = src/gpio.c src/HAL.c src/util.c
OBJ = $(SRC:.c=.o)
LIBNAME = gpio

all: static shared

static: $(OBJ)
	ar rcs lib$(LIBNAME).a $(OBJ)

shared: $(OBJ)
	$(CC) -shared -o lib$(LIBNAME).so $(OBJ) -lfdt

clean:
	rm -f $(OBJ) lib$(LIBNAME).a lib$(LIBNAME).so
