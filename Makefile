compile:
	gcc -O3 -o orb orb.c pool.c -lSDL2 -lSDL2main

debug:
	clear
	gcc -g -Wall -o orb orb.c pool.c -lSDL2 -lSDL2main

clean:
	rm examples/*.rom
