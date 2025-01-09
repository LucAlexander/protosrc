run:
	clear
	gcc -g -Wall -o orb orb.c pool.c -lSDL2 -lSDL2main
	./orb

compile:
	gcc -O3 -o orb orb.c pool.c -lSDL2 -lSDL2main


clean:
	rm examples/*.rom
