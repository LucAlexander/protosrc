run:
	clear
	gcc -g -Wall -o orb orb.c pool.c
	./orb

compile:
	gcc -g -Wall -o orb orb.c orb.c


clean:
	rm *.rom
