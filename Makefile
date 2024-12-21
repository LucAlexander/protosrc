run:
	clear
	gcc -g -Wall -o orb orb.c pool.c
	./orb

compile:
	gcc -g -Wall -o orb orb.c pool.c


clean:
	rm *.rom
