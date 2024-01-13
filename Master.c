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
    int shm_fd;

    shm_fd = shm_open (MAP_ADDR, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) perror ("shm open");


    if (ftruncate (shm_fd, 2 * sizeof (double)) < 0) perror ("truncate");

    int sh_kb_fd = shm_open (KB_ADDR, O_CREAT | O_RDWR, 0666);
    if (sh_kb_fd < 0) perror ("shm kb open");

    if (ftruncate (sh_kb_fd, sizeof (int)) < 0) perror ("ftruncate");

    sem_t * map_sem = sem_open (MAP_SEM, O_CREAT, 0666, 1);

    sem_t * kb_sem = sem_open (KB_SEM, O_CREAT, 0666, 1);

    const int proc_numb = 3, proc_kons = 2;
    int null_wait;
    int logfd [proc_numb];

    for (int i = 0; i < proc_numb; i++) {
        mkfifo (log_file [i], 0666);
        logfd [i] = open (log_file [i], O_CREAT | O_RDWR, 0666);
        if (logfd [i] < 0) perror ("log open");
    }
    
    
    int proc_pid [proc_numb], res;

    //need to create pipes between server and processes
    int out_fds  [proc_numb - 1] [2];
    int in_fds [proc_numb - 1] [2];
    for (int i = 0; i < proc_numb - 1; i++) {
        if (pipe (out_fds [i]) < 0) {
            perror ("pipe generation");
            exit (EXIT_FAILURE);
        }
        if (pipe (in_fds [i]) < 0) {
            perror ("pipe generation");
            exit (EXIT_FAILURE);
        }
    }
    /*for (int i = 0; i < proc_numb - 1; i++) {
        printf ("%d %d\n", fds [i] [0], fds [i] [1]);
    }*/
    char server_fd [proc_numb - 1] [20];//10 is max number of digits for int
    char process_fd [proc_numb - 1] [20]; //ons string per process
    //in the server argument there is going to be a series of pairs of file directories, the first one is the request reception (read from)
    for (int i = 0; i < proc_numb - 1; i++) {
        sprintf (server_fd [i], "%d %d", out_fds [i] [0], in_fds [i] [1]);
        //printf ("%s", server_fd [i]);
        sprintf (process_fd [i], "%d %d", in_fds [i] [0], out_fds [i] [1]);
    }

    

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
        char * arglist1 [] = {"./drone_dyn", process_fd [0], NULL};
        if (execvp (arglist1 [0], arglist1) < 0) {
            perror ("execvp 1");
            exit (EXIT_FAILURE);
        }
    }

    proc_pid [1] = fork ();
    if (proc_pid [1] < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }
    if (proc_pid [1] == 0) {
        char * arglist2 []= {"konsole", "-e", "./map_displayer", process_fd [1], NULL};
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
        char * serverarglist [] = {/*"konsole", "-e", */"./BB_server", server_fd [0], server_fd [1], NULL};
        if (execvp (serverarglist [0], serverarglist) < 0) {
            perror ("exec 4");
            exit (EXIT_FAILURE);
        }
    }

    if ((res = fork ()) < 0) {
        perror ("fork");
        exit (EXIT_FAILURE);
    }

    if (res == 0) {
        char * konsargwtcdg [] = {"./watchdog", NULL};

        if (execvp (konsargwtcdg [0], konsargwtcdg) < 0) {
            perror ("execvp 5");
            exit (EXIT_FAILURE);
        }
    } 
    
    for (int i = 0; i < proc_numb -1; i++) {
        close (out_fds [i] [0]);
        close (in_fds [i] [1]);
        close (out_fds [i] [1]);
        close (in_fds [i] [0]);
    }
    
    int term_child;

    term_child = waitpid (proc_pid [2], &null_wait, 0);
    if (term_child < 0) perror ("wait");

    printf ("the blackboard process has terminated\n");

    term_child = waitpid (proc_pid [0], &null_wait, 0);
    if (term_child < 0) perror ("wait");

    printf ("the drone process has terminated\n");
    term_child = waitpid (proc_pid [1], &null_wait, 0);
    if (term_child < 0) perror ("wait");

    printf ("the map process has terminated\n");

    term_child = waitpid (res, &null_wait, 0);
    if (term_child < 0) perror ("wait");

    printf ("the watchdog process has terminated\n");    

    printf ("all processes have terminated\n");


    /*for (int i = 0; i < proc_numb; i++) {
        if (proc_pid [i] == term_child) continue;
        if (kill (proc_pid [i], SIGKILL) < 0) {
            perror ("kill");
        }
        close (logfd [i]);
    }*/
    //f (res != term_child) {
    /*if (kill (res, SIGKILL) < 0) {
        perror ("kill");
    }*/
    //}
    printf ("game finished!\n");
    printf ("bye!\n");

    

    sem_unlink (MAP_SEM);
    sem_unlink (KB_SEM);
    shm_unlink (MAP_ADDR);
    shm_unlink (KB_ADDR);

    exit (EXIT_SUCCESS);
    return 0;
}