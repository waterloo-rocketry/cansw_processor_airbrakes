CC = gcc
INCLUDES = -I ../../include
CFLAGS = -Wall

all: firmware_update

clean:
	rm -f main.o
	rm -f firmware_update
	cd ../.. && make clean

firmware_update: main.o proglib
	$(CC) -o firmware_update main.o -L ../../build -lvnc -lpthread

proglib:
	cd ../.. && make

main.o: main.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

