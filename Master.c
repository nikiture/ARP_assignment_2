#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <sys/wait.h>
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <signal.h>
#include "world_info.h"

//const char * logfile = "process_status";


int main () {




    const int proc_numb = 6, proc_kons = 2;
    int null_wait;
    int logfd [proc_numb];

    for (int i = 0; i < proc_numb - 1; i++) {
        mkfifo (log_file [i], 0666);
        logfd [i] = open (log_file [i], O_CREAT | O_RDWR, 0666);
        if (logfd [i] < 0) {
            perror ("log open");
            printf ("pipe %d\n", i);
        }
    }
    
    
    int proc_pid [proc_numb];

    //need to create pipes between server and processes
    int out_fds  [proc_numb - 2] [2];
    int in_fds [proc_numb - 2] [2];
    for (int i = 0; i < proc_numb - 2; i++) {
        if (pipe (out_fds [i]) < 0) {
            perror ("pipe generation");
            exit (EXIT_FAILURE);
        }
        if (pipe (in_fds [i]) < 0) {
            perror ("pipe generation");
            exit (EXIT_FAILURE);
        }
    }
    //pipe from keyboard reader (which is the map process) to dynamics process
    int kb_to_din_fd [2];
    if (pipe (kb_to_din_fd) < 0) {
        perror ("pipe generation");
        exit (EXIT_FAILURE);
    }

    char server_fd [proc_numb - 2] [20];//10 is max number of digits for int
    char process_fd [proc_numb - 2] [20]; //one string per process excluding watchdog
    char kb_din_fd [10];
    //in the server argument there is going to be a series of pairs of file directories, the first one is the request reception (read from)
    for (int i = 0; i < proc_numb - 2; i++) {
        sprintf (server_fd [i], "%d %d", out_fds [i] [0], in_fds [i] [1]);

        sprintf (process_fd [i], "%d %d", in_fds [i] [0], out_fds [i] [1]);
    }
    sprintf (kb_din_fd, "%d", kb_to_din_fd [0]);

    

    printf ("welcome to the game! here are the commands for the game while the processes are loading\n");
    printf ("use the w, e, r, s, d, f, x, c and v keys to control the drone\n");
    printf ("press q to quit the game, z to restart the game (placing the drone at the center of the map\n");
    sleep (5);

    proc_pid [0] = fork ();
    if (proc_pid [0] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [0] == 0) {
        char * arglist1 [] = {/*"konsole", "-e", */"./drone_dyn", process_fd [0], kb_din_fd, NULL};
        if (execvp (arglist1 [0], arglist1) < 0) {
            perror ("execvp 1");
            exit (EXIT_FAILURE);
        }
    }
    sprintf (kb_din_fd, "%d", kb_to_din_fd [1]);

    proc_pid [1] = fork ();
    if (proc_pid [1] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [1] == 0) {
        char * arglist2 []= {"konsole", "-e", "./map_displayer", process_fd [1], kb_din_fd, NULL};
        if (execvp (arglist2 [0], arglist2) < 0) {
            perror ("execvp 3");
            exit (EXIT_FAILURE);
        }
    }
    proc_pid [2] = fork ();
    if (proc_pid [2] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [2] == 0) {
        char * serverarglist [] = {"./BB_server", server_fd [0], server_fd [1], server_fd [2], server_fd [3], NULL};
        if (execvp (serverarglist [0], serverarglist) < 0) {
            perror ("exec 4");
            exit (EXIT_FAILURE);
        }
    }
    proc_pid [3] = fork ();
    if (proc_pid [3] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [3] == 0) {
        char * obsarglist [] = {"./obstacle_generator", process_fd [2], NULL};
        if (execvp (obsarglist [0], obsarglist) < 0) {
            perror ("exec obs");
            exit (EXIT_FAILURE);
        }
    }
    proc_pid [4] = fork ();
    if (proc_pid [4] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [4] == 0) {
        char * targarglist [] = {"./target_generator", process_fd [3], NULL};
        if (execvp (targarglist [0], targarglist) < 0) {
            perror ("exec targ");
            exit (EXIT_FAILURE);
        }
    }

    if ((proc_pid [5] = fork ()) < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    char kons_map_pid [10];

    sprintf (kons_map_pid, "%d", proc_pid [1]);

    if (proc_pid [5] == 0) {
        char * konsargwtcdg [] = {"./watchdog", kons_map_pid, NULL}; //sending to watchdog also pids of konsole to properly close it in case of crash

        if (execvp (konsargwtcdg [0], konsargwtcdg) < 0) {
            perror ("execvp 5");
            exit (EXIT_FAILURE);
        }
    } 
    
    
    
    int term_child;

    term_child = waitpid (proc_pid [5], &null_wait, 0);
    if (term_child < 0) perror ("wait");

    printf ("the watchdog process has terminated\n"); 

    
    printf ("killing all other processes\n");

    for (int i = 0; i < proc_numb; i++) {
        kill (proc_pid [i], SIGKILL);
    }
    for (int i = 0; i < proc_numb -1; i++) {
        close (out_fds [i] [0]);
        close (in_fds [i] [1]);
        close (out_fds [i] [1]);
        close (in_fds [i] [0]);
    }
       
    printf ("game finished!\n");
    printf ("bye!\n");

    for (int i = 0; i < proc_numb; i++) {
        close (logfd [i]);
    }

    exit (EXIT_SUCCESS);
    return 0;
}