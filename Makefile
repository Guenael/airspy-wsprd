CC = gcc
CFLAGS= -Wall -O3 -ffast-math -std=gnu99
LDFLAGS = -L/usr/lib
LIBS = -lusb-1.0 -lairspy -lpthread -lfftw3 -lcurl -lm

OBJS = airspy_wsprd.o wsprd.o wsprsim_utils.o wsprd_utils.o tab.o fano.o nhash.o

%.o: %.c
	${CC} ${CFLAGS} -c $< -o $@

airspy_wsprd: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o airspy_wsprd wspr_wisdom.dat hashtable.txt