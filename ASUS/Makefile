EXENAME  = uart
OBJS     = UART.o
CXX      = gcc
CXXFLAGS = -g -pthread -lc
LD       = gcc
LDFLAGS  = -g -lc -lpthread

all: $(EXENAME)

$(EXENAME) : $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o $(EXENAME)

uart.o : UART.c UART.h
	$(CXX) $(CXXFLAGS) UART.c
clean:
	-rm -f *.o $(EXENAME)
