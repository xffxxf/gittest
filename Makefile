
include ../pdstars.mak

SRC=./src/UHFHFCTAETEVApi.c ./src/crc.c  ./src/protocol.c ./src/dft.c ./src/ae_view_convert.c 

all:clean libUHFHFCTAETEVApi.so  testtest

libUHFHFCTAETEVApi.so:
	$(CC) $(CFLAG_DYNAMIC)  $(LDFLAGS) $(CFLAGS) -I ../include/ -I ../uart/src/ -lm  -luart  -lgpio $(SRC) -o $@
	cp $@ ../lib

testtest:test/shtest.c
	$(CC) -I src/ $(LDFLAGS) $(CFLAGS) $^ -o $@ -lm -L. -lUHFHFCTAETEVApi  -luart  -lgpio
	cp $@ ../bin-bash

clean:
	rm -f *.o *.so UHFHFCTAETEVtest testtest
