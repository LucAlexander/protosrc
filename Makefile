run:
	clear
	gcc -g -Wall -o orb orb.c pool.c -lSDL2 -lSDL2main
	./orb

compile:
	gcc -g -Wall -o orb orb.c pool.c -lSDL2 -lSDL2main


clean:
	rm *.rom
