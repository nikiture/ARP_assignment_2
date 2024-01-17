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

#define DRONE_MARKER 'X'
#define OBSTACLE_MARKER 'O'

const double framerate = 50;
int logfd = 0;


void printerror (const char * errmsg) {
    endwin ();
    perror (errmsg);
    if (logfd > 0) close (logfd);
    int time_left = sleep (5);
    while (time_left > 0) {
        time_left = sleep (time_left);
    }
    exit (EXIT_FAILURE);
}
const int log_id = 1; //position in log_file where pid is written

        
void watchdog_req (int signumb) {

    if (signumb == SIGUSR1) {

        logfd = open (log_file [log_id], O_WRONLY, 0666);
        if (logfd < 0) {
            printerror ("log file open");
        }

        char logdata [10];

        if (sprintf (logdata, "%c", 'r') < 0) {
            printerror ("log formatting");
        }
        if (write (logfd, logdata, sizeof (logdata)) < 0) {
            printerror ("log write");
        }
        close (logfd);

    }
}

void read_drone_from_BB (int in_fd, int out_fd, double * drone_pos, char * IO_msg, struct timespec * t, sigset_t * sigmask) {

    sprintf (IO_msg, "%c", 'd');

    if (write (out_fd, IO_msg, strlen (IO_msg) + 1) < 0) {
        printerror ("BB write");
        exit (EXIT_FAILURE);
    }

    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    int syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 240) < 0) {
        printerror ("BB read");
        exit (EXIT_FAILURE);
    }

    if (sscanf (IO_msg, "%lf %lf", &drone_pos [0], &drone_pos [1]) < 0) {
        printerror ("drone deformatting");
    }
}

void request_obs (int in_fd, int out_fd, int * obs_placed, double obs_pos [obs_num] [2], char * IO_msg, char * tmp_msg, struct timespec * t, sigset_t * sigmask) {

    if (sprintf (IO_msg, "%c", 'r') < 0) {
        printerror ("request formatting");
    }

    int syscall_res = write (out_fd, IO_msg, strlen (IO_msg) + 1);

    if (syscall_res < 0) {
        printerror ("request write");
    }

    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 240) < 0) {
        printerror ("request read");
    }

    if (IO_msg [0] == 'n') return;

    sscanf (IO_msg, "%*c%d%*c%s", obs_placed, tmp_msg); //in IO_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    strcpy (IO_msg, tmp_msg);

    if (*obs_placed > obs_num) *obs_placed = obs_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &obs_pos [0] [0], &obs_pos [0] [1], tmp_msg);
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *obs_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &obs_pos [i] [0], &obs_pos [i] [1], tmp_msg);
        strcpy (IO_msg, tmp_msg);
    }

    for (int i = 0; i < *obs_placed; i++) {
        obs_pos [i] [1] *= -1; 
        obs_pos [i] [1] += MAP_Y_SIZE;
    }
}

int main (int argc, char ** argv) {


    signal (SIGUSR1, &watchdog_req);


    int fd_in_server, out_fd_server;



    int count = 0;
    const int maxcount = 2;
    if (argc < 3) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    sscanf (argv [1], "%d %d", &fd_in_server, &out_fd_server);
    
    
    int fd_drone;
    sscanf (argv [2], "%d", &fd_drone);
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

    if (write (logfd, logdata, strlen (logdata) + 1) < 0) {
        printerror ("watchdog write");
    }

    double drone_pos [2], old_pos [2];
    double obs_pos [obs_num] [2];
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i] [0] = 10;
        obs_pos [i] [1] = 10;
    }
    char IO_msg [240], tmp_str [240];
    int obs_placed = obs_num;

    int mapsize [2];
    int kb_res;

    if (read (fd_in_server, IO_msg, 160) < 0) {
        printerror ("wait read");
    }
    
    
    initscr(); 

    

    struct timespec t;
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
    if (has_colors () == 1) {
        if (init_color (COLOR_YELLOW, 1000, 568, 0) < 0) { //redefining COLOR_YELLOW to be orange
            printerror ("color_change");
        }
        init_pair (1, COLOR_BLUE, COLOR_BLACK); //first is characters, second is background
        init_pair (2, COLOR_WHITE, COLOR_BLACK);
        init_pair (3, COLOR_YELLOW, COLOR_BLACK);

    }

    sigset_t select_mask, orig_mask;
    sigaddset (&select_mask, SIGUSR1);
    

    while (1) {

        //read kb presses and send them to dynamics process

        kb_res = getch ();


        if (sprintf (IO_msg, "%d", kb_res) < 0) {
            printerror ("drone sprintf");
        }
        if (write (fd_drone, IO_msg, strlen (IO_msg) + 1) < 0) {
            printerror ("drone write");
        }

        if (kb_res == 'q') {
            sprintf (logdata, "%c", 'q');
            //write to blackboard (which then writes to obstacle generator)
            write (out_fd_server, logdata, strlen (logdata) + 1);
            //write to watchdog
            write (logfd, logdata, strlen (logdata) + 1);
            sleep (3);
            usleep (50000);//sleep should be interrupted because of the watchdog monitoring, after that it waits a little bit more to let the watchdog notice the quit message on the logpipe
            close (logfd);
            exit (EXIT_SUCCESS);
        }
        if (kb_res == 'z') {
            if (sprintf (IO_msg, "%c", 'z') < 0) {
                printerror ("sprintf");
            }
            if (write (out_fd_server, IO_msg, strlen (IO_msg) + 1) < 0) {
                printerror ("write");
            }
            //read to wait for server acknowledgement (else risk of softlock on read)
            if (read (fd_in_server, IO_msg, 160) < 0) {
                printerror ("BB reset response read");
            }
        }

        syscall_res = clear ();
        if (syscall_res == ERR) {
            printf ("issues clearing\n\r");
            printerror ("addch clear");
        }

        read_drone_from_BB (fd_in_server, out_fd_server, drone_pos, IO_msg, &t, &select_mask);

        request_obs (fd_in_server, out_fd_server, &obs_placed, obs_pos, IO_msg, tmp_str, &t, &select_mask);

        //adjustemnts to drone position to match with ncurses: positive going downward, maximum at 0
        drone_pos [1] *= -1; 
        drone_pos [1] += MAP_Y_SIZE;

        //limitations to positions put just in case to still visualize the drone, but they should not be necessary with the wall forces implemented
        if (drone_pos [0] < 0) drone_pos [0] = 0;
        if (drone_pos [0] > MAP_X_SIZE) drone_pos [0] = MAP_X_SIZE;
        if (drone_pos [1] < 0) drone_pos [1] = 0;
        if (drone_pos [1] > MAP_Y_SIZE) drone_pos [1] = MAP_Y_SIZE;

        //printing stuff on terminal, blocking SIGUSR1 to avoid errors in the functions
        sigprocmask (SIG_SETMASK, &select_mask, &orig_mask);
        syscall_res = wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE); //makes sure the playground (visualized using box () below) is of the expected size
        if (syscall_res == ERR) {
            printf ("issues resizing\n\r");
            printerror ("resizing");
        }
        
        //set character color to white for border, the re-set it back to default
        

        attron (COLOR_PAIR (2));
        box (stdscr, '|', '-');
        attroff (COLOR_PAIR (2));

    
        attron (COLOR_PAIR (1));
        syscall_res = mvprintw ((int) round (drone_pos [1]), (int) round (drone_pos [0]), "%c", DRONE_MARKER);
        attroff (COLOR_PAIR (1));
        attron (COLOR_PAIR (3));                
        for (int i = 0; i < obs_placed; i++) {
            syscall_res = mvprintw ((int) round (obs_pos [i] [1]), (int) round(obs_pos [i] [0]), "%c", OBSTACLE_MARKER);
        }
        attroff (COLOR_PAIR (3));
        
        refresh ();
        sigprocmask(SIG_SETMASK, &orig_mask, NULL);

        count = (count + 1) % maxcount;

        usleep (SEC_TO_USEC / framerate);


    }
    return 0;
}