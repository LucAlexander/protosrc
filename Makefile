run:
	clear
	gcc -g -Wall -o machine main.c pool.c
	./machine

compile:
	gcc -g -Wall -o machine main.c pool.c


clean:
	rm *.rom
