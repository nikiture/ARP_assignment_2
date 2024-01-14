#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>

#include "world_info.h"
#include <errno.h>

const char DRONE_MARKER = 'X';
const double framerate = 50;
int logfd = 0;


void printerror (const char * errmsg) {
    endwin ();
    perror (errmsg);
    printf ("issues in %s\n%d\n", errmsg , errno);
    //refresh ();
    if (logfd > 0) close (logfd);
    int time_left = sleep (30);
    while (time_left > 0) {
        time_left = sleep (time_left);
    }
    exit (EXIT_FAILURE);
}
const int log_id = 1; //position in log_file where pid is written

        
void watchdog_req (int signumb) {

    if (signumb == SIGUSR1) {
        //printw ("responding to watchdog\n\r");
        //refresh();
        logfd = open (log_file [log_id], O_WRONLY, 0666);
        if (logfd < 0) {
            printerror ("log file open");
        }

        int pid = getpid ();
        char logdata [10];

        if (sprintf (logdata, "%c", 'r') < 0) {
            printerror ("log formatting");
        }
        if (write (logfd, logdata, sizeof (logdata)) < 0) {
            printerror ("log write");
        }
        close (logfd);
        //printw ("finished responding to server\n\r");
    }
}

void read_drone_from_BB (int in_fd, int out_fd, double * drone_pos, char * IO_msg) {
    sprintf (IO_msg, "%c", 'd');
    if (write (out_fd, IO_msg, strlen (IO_msg) + 1) < 0) {
        printerror ("BB write");
        exit (EXIT_FAILURE);
    }
    if (read (in_fd, IO_msg, 80) < 0) {
        printerror ("BB read");
        exit (EXIT_FAILURE);
    }
    printw ("displayer: %s\n\r", IO_msg);
    if (sscanf (IO_msg, "%lf %lf", &drone_pos [0], &drone_pos [1]) < 0) {
        printerror ("drone deformatting");
    }
    printw ("%lf %lf\n\r", drone_pos [0], drone_pos [1]);
    refresh ();
}

int main (int argc, char ** argv) {
    int fd_in_server, out_fd_server;



    int count = 0;
    const int maxcount = 10;
    sscanf (argv [1], "%d %d", &fd_in_server, &out_fd_server);
    
    signal (SIGUSR1, watchdog_req);

    sem_t * map_semaph = sem_open (MAP_SEM, O_CREAT, 0666, 1);
    if (map_semaph == SEM_FAILED) {
        printerror ("semaphore creation");
    }

    if (sem_init (map_semaph, 0, 1) < 0) printerror ("semaphore initialisation");

    sem_t * kb_sem = sem_open (KB_SEM, O_CREAT, 0666, 1);
    if (kb_sem == SEM_FAILED) {
        printerror ("semaphore creation kb");
    }

    if (sem_init (kb_sem, 0, 1) < 0) printerror ("semaphore kb creation");

    int fd_drone = shm_open (MAP_ADDR, O_RDWR, 0666);
    if (fd_drone < 0) printerror ("shared memory opening");

    void * drone_ptr = mmap (0, 2 * sizeof (double), PROT_READ | PROT_WRITE, MAP_SHARED, fd_drone, 0);
    if (drone_ptr == MAP_FAILED) printerror ("shared memory mapping");

    int kb_fd = shm_open (KB_ADDR, O_RDWR, 0666);
    if (kb_fd < 0) printerror ("shared memory opening kb");

    void * kb_ptr = mmap (0, sizeof (int), PROT_WRITE | PROT_READ, MAP_SHARED, kb_fd, 0);
    if (kb_ptr == MAP_FAILED) printerror ("shared memory kb mapping");

    int logfd = open (log_file [log_id], O_WRONLY, 0666);
    if (logfd < 0) printerror ("log file open");

    char logdata [10];
    int syscall_res;

    int pid = getpid ();

    if (sprintf (logdata, "%d", pid) < 0) printerror ("pid log formatting");

    if (write (logfd, logdata, sizeof (logdata)) < 0) {
        printerror ("watchdog write");
    }

    double drone_pos [2], old_pos [2];
    char IO_msg [80];

    int mapsize [2];
    int kb_res;
    initscr();

    printf ("welcome to the game! here are the commands for the game\n\r");
    printf ("use the w, e, r, s, d, f, x, c and v keys to control the drone\n\r");
    printf ("press q to quit the game, z to restart the game\n\r");

    //usleep (50000); //waits a bit of time (50 milliseconds) in order to let the drone dynamics process write the first position on the shared memory

    struct timespec start_time;
    struct timespec end_time;
    struct timespec delta_time, rem_time;
    long int time_to_sleep;
    long int nsec_diff;
    void * memcopy_res = NULL;
    
    wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE);
    resizeterm (MAP_Y_SIZE, MAP_X_SIZE);
    cbreak();
    noecho();
    curs_set (0);
    nodelay (stdscr, 0);
    wtimeout (stdscr, 5);
    start_color ();
    //int colorable = has_colors ();
    if (has_colors () == 1) {
        int color_change = can_change_color ();
        //printf ("%d\n", color_change);
        //sleep (4);  
        init_pair (1, COLOR_GREEN, COLOR_BLACK); //first is characters, second is background
        init_pair (2, COLOR_WHITE, COLOR_BLACK);
        //attron (COLOR_PAIR (1));
        bkgd (COLOR_PAIR (1));
    }
    /*printw ("%d", has_colors ());
    refresh ();*/
    //sleep (20);
    sigset_t select_mask, orig_mask;
    sigaddset (&select_mask, SIGUSR1);

    while (1) {

        //read kb presses and send them to dynamics process
        kb_res = getch ();
        if (sem_wait (kb_sem) < 0) printerror ("kb semaphore taking");

        memcopy_res = memcpy (kb_ptr, &kb_res, sizeof (int));
        if (memcopy_res == NULL) printerror ("shared memory kb write");

        if (sem_post (kb_sem) < 0) printerror ("kb semaphore release");

        if (kb_res == 'q') {
            close (logfd);
            exit (EXIT_SUCCESS);
        }

                            /*receive from shared memory drone position*/
        /*if (sem_wait (map_semaph) < 0) printerror ("semaphore taking");

        memcopy_res = memcpy (drone_pos, drone_ptr, 2 * sizeof (double));

        if (memcopy_res == NULL) printerror ("shared memory reading");
        
        if (sem_post (map_semaph) < 0) printerror ("semaphore releasing");*/
        if (count == maxcount / 2) {
        //do {
            read_drone_from_BB (fd_in_server, out_fd_server, drone_pos, IO_msg);
        //adjustemnts to drone position to match with ncurses: positive going downward, maximum at 0
            drone_pos [1] *= -1; 
            drone_pos [1] += MAP_Y_SIZE;
        //limitations to positions put just in case to still visualize the drone, but they should not be necessary with the wall forces implemented
            if (drone_pos [0] < 0) drone_pos [0] = 0;
            if (drone_pos [0] > MAP_X_SIZE) drone_pos [0] = MAP_X_SIZE;
            if (drone_pos [1] < 0) drone_pos [1] = 0;
            if (drone_pos [1] > MAP_Y_SIZE) drone_pos [1] = MAP_Y_SIZE;
            sigprocmask (SIG_SETMASK, &select_mask, &orig_mask);
            syscall_res = wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE); //makes sure the playground (visualized using box () below) is of the expected size
            if (syscall_res == ERR) {
                printf ("issues resizing\n\r");
                printerror ("resizing");
            }
            syscall_res = clear ();
            if (syscall_res == ERR) {
                printf ("issues clearing\n\r");
                printerror ("addch clear");
            }
            //set character color to white for border, the re-set it back to color assigned to drone's character

            attroff (COLOR_PAIR (1));
            attron (COLOR_PAIR (2));
            box (stdscr, '|', '-');
            attroff (COLOR_PAIR (2));
            attron (COLOR_PAIR (1));

            //update drone position on the map displayed
            //do {
                //syscall_res = printw ("prova di stampa\n\r");
                /*if (syscall_res == ERR) {

                    if (errno == EINTR) continue;
                    
                    else printerror ("test print -1");
                }*/
                //syscall_res = printw ("%lf3 %lf3\n\r", drone_pos [0], drone_pos [1]);
                /*if (syscall_res < 0) {

                    if (errno == EINTR) continue;
                    
                    else printerror ("test print");
                }*/

                //refresh ();
                //sleep (10);
                syscall_res = mvprintw ((int) round (drone_pos [1]), (int) round (drone_pos [0]), "%c", DRONE_MARKER);
                /*syscall_res = mvprintw ((int) MAP_X_SIZE/2, (int) MAP_Y_SIZE / 2, "%c", 'T');
                if (syscall_res == ERR) {
                    //if (errno != 4) {
                        printf ("issues placing drone\n\r");
                        printerror ("addch");
                    //}
                }*/
                
            //} while (errno != 4);*/

            refresh ();
            sigprocmask(SIG_SETMASK, &orig_mask, NULL);
        //} while (errno != EINTR);

        }

        count = (count + 1) % maxcount;
        //sleep (2);
        usleep (SEC_TO_USEC / framerate);


    }
    return 0;
}