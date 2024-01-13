all: Master drone_dyn watchdog map_displayer BB_server add_param

clean: clean_binaries clean_logs

clean_binaries:
	rm build/Master build/map_displayer build/drone_dyn build/watchdog build/BB_server build/parameters.txt

clean_logs:
	rm build/log_results.txt

start_build: build
	mkdir build

Master: Master.c
	gcc Master.c -o build/Master

drone_dyn: drone_dyn.c
	gcc drone_dyn.c -o build/drone_dyn -lm -lrt -lpthread

watchdog: watchdog.c 
	gcc watchdog.c -o build/watchdog
	

map_displayer: map_displayer.c
	gcc map_displayer.c -o build/map_displayer -lm -lrt -lpthread -lncurses

BB_server: BB_server.c
	gcc BB_server.c -o build/BB_server

add_param: 
	cp parameters.txt build/parameters.txt
