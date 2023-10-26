#B4: Emre Eryilmaz, David Gögle, Ved Varshney
# add entries for all the additional programs
all: sum_numbers hello manipulate_two_numbers_loop manipulate_two_numbers 

# compile hello_world.c
hello: 
	gcc -o hello_world hello_world.c

#linking manipulate_two_numbers_loop.o functions.o
manipulate_two_numbers_loop: manipulate_two_numbers_loop.o functions.o 
	gcc manipulate_two_numbers_loop.o functions.o -o manipulate_two_numbers_loop

#compile manipulate_two_numbers_loop
manipulate_two_numbers_loop.o:
	gcc -c manipulate_two_numbers_loop.c 

#linking manipulate_two_numbers.o with functions.o
manipulate_two_numbers: manipulate_two_numbers.o functions.o 
	gcc manipulate_two_numbers.o functions.o -o manipulate_two_numbers

#compile manipulate_two_numbers.c
manipulate_two_numbers.o:
	gcc -c manipulate_two_numbers.c

# linking of sum_numbers.o with functions.o
sum_numbers: sum_numbers.o functions.o
	gcc sum_numbers.o functions.o -o sum_numbers

# compile sum_numbers.c	
sum_numbers.o: sum_numbers.c
	gcc -c sum_numbers.c
	
# compile functions.c
functions.o: functions.h functions.c
	gcc -c functions.c

# remove generated files and programs
clean:
	rm functions.o sum_numbers.o sum_numbers manipulate_two_numbers.o manipulate_two_numbers manipulate_two_numbers_loop.o manipulate_two_numbers_loop
