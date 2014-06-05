
all: libads129x.a ads129x-setup

libads129x.o: libads129x.h

libads129x.a: libads129x.o
	$(AR) rcs $@ $<

ads129x-setup: ads129x-setup.c libads129x.a libads129x.h
	$(CC) $(CFLAGS) -o $@ $@.c -L. -lads129x
