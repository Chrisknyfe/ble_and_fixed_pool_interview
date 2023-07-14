


all: proprietary_ble.c
	gcc -g -Wall -lm -o proprietary_ble proprietary_ble.c
	

clean:
	rm proprietary_ble

test: all
	./proprietary_ble


debug: all
	gdb proprietary_ble

