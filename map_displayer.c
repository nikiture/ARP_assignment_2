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
char Targ_marker [targ_num] [2];  //each target has up to two digits identifying it

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

void read_drone_from_BB (int in_fd, int out_fd, struct position * drone_pos, char * IO_msg, struct timespec * t, sigset_t * sigmask) {

    sprintf (IO_msg, "%c", 'd');

    if (write (out_fd, IO_msg, strlen (IO_msg) + 1) < 0) {
        printerror ("BB write");
        exit (EXIT_FAILURE);
    }

    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    int syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect drone");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 240) < 0) {
        printerror ("BB read");
        exit (EXIT_FAILURE);
    }

    if (sscanf (IO_msg, "%lf %lf", &(drone_pos->x), &(drone_pos->y)) < 0) {
        printerror ("drone deformatting");
    }

    //adjustemnts to drone position to match with ncurses: positive going downward, maximum at 0
    drone_pos->y *= -1; 
    drone_pos->y += MAP_Y_SIZE;

    //limitations to positions put just in case to still visualize the drone, but they should not be necessary with the wall forces implemented
    if (drone_pos->x < 0) drone_pos->x = 0;
    if (drone_pos->x > MAP_X_SIZE) drone_pos->x = MAP_X_SIZE;
    if (drone_pos->y < 0) drone_pos->y = 0;
    if (drone_pos->y > MAP_Y_SIZE) drone_pos->y = MAP_Y_SIZE;
}

void request_obs (int in_fd, int out_fd, int * obs_placed, struct position obs_pos [obs_num], char * IO_msg, char * tmp_msg, struct timespec * t, sigset_t * sigmask) {

    if (sprintf (IO_msg, "%c", 'o') < 0) {
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
    if (syscall_res < 0) printerror ("pselect obs");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 240) < 0) {
        printerror ("request read");
    }

    if (IO_msg [0] == 'n') return; //obstacles not changed, message deformatting not to be done

    sscanf (IO_msg, "%*c%d%*c%s", obs_placed, tmp_msg); //in IO_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    strcpy (IO_msg, tmp_msg);

    if (*obs_placed > obs_num) *obs_placed = obs_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(obs_pos [0].x), &(obs_pos [0].y), tmp_msg);
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *obs_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(obs_pos [i].x), &(obs_pos [i].y), tmp_msg);
        strcpy (IO_msg, tmp_msg);
    }

    for (int i = 0; i < *obs_placed; i++) {
        obs_pos [i].y *= -1; 
        obs_pos [i].y += MAP_Y_SIZE;
    }
}


void request_targ (int in_fd, int out_fd, int * targ_placed, struct position targ_pos [targ_num], char * IO_msg, char * tmp_msg, struct timespec * t, sigset_t * sigmask) {
    if (sprintf (IO_msg, "%c", 't') < 0) {
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
    if (syscall_res < 0) printerror ("pselect obs");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 240) < 0) {
        printerror ("request read");
    }

    /*fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    /*do {
        syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
        if (syscall_res < 0) {
            if (errno != EINTR) printerror ("pselect target");
        }
    } while (syscall_res < 0);
    if (syscall_res == 0) return;*/
    /*do {
        syscall_res = read (in_fd, IO_msg, 240);
        if (syscall_res < 0 && errno != EINTR) {
            printerror ("request read");
        }
    } while (syscall_res < 0);*/

    if (IO_msg [0] == 'n') return; //targets not changed, message deformatting not to be done

    sscanf (IO_msg, "%*c%d%*c%s", targ_placed, tmp_msg); //in IO_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    strcpy (IO_msg, tmp_msg);

    if (*targ_placed > targ_num) *targ_placed = targ_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(targ_pos [0].x), &(targ_pos [0].y), tmp_msg);
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *targ_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(targ_pos [i].x), &(targ_pos [i].y), tmp_msg);
        strcpy (IO_msg, tmp_msg);
    }

    for (int i = 0; i < *targ_placed; i++) {
        targ_pos [i].y *= -1; 
        targ_pos [i].y += MAP_Y_SIZE;
    }
}

void request_current_target (int out_fd, int in_fd, int * curr_targ, char IO_msg [240], int * reset, sigset_t * selectmask) {
    /*if (sprintf (IO_msg, "%c", 'c') < 0) 
        printerror ("first target request message");
    if (write (out_fd, IO_msg, strlen (IO_msg) + 1) < 0) 
        printerror ("first target request");/**/
    if (dprintf (out_fd, "%c\n", 'c') < 0) printerror ("current target request");
    //printw ("sent request, waiting for response\n\r");
    //refresh ();
    /*fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);*/
    int syscall_res;
    /*do {
        syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, selectmask);
        if (syscall_res < 0) {
            if (errno != EINTR) printerror ("first target select");
        }
    } while (syscall_res < 0);*/
    do {
        syscall_res = read (in_fd, IO_msg, 240);
        if (syscall_res < 0) {
            if (errno == EINTR) {
                errno = 0;
                continue;
            }
            printerror ("first target response");  
        } 
    } while (syscall_res < 0);/**/
    //printw ("response received, extracting content\n\r");
    //refresh ();
    
    //printerror (IO_msg);    
    if (sscanf (IO_msg, "%d", curr_targ) < 0) 
        printerror ("first target decoding");
    //printerror (IO_msg);
}



int main (int argc, char ** argv) {


    signal (SIGUSR1, &watchdog_req);


    for (int i = 0; i< targ_num; i++) {
        sprintf (Targ_marker [i], "%d", i);
    }
    int curr_targ = 0; //identifier of currentlyfirst target to pick in the order

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

    int logfd = open (log_file [log_id], O_WRONLY, 0666);
    if (logfd < 0) printerror ("log file open");

    char logdata [10];
    int syscall_res;

    int pid = getpid ();
    if (pid < 0) printerror ("pid reception");

    if (sprintf (logdata, "%d", pid) < 0) 
        printerror ("pid log formatting");

    if (write (logfd, logdata, strlen (logdata) + 1) < 0) {
        printerror ("watchdog write");
    }

    struct position drone_pos;
    struct position obs_pos [obs_num];
    struct position targ_pos [targ_num];
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i].x = 10;
        obs_pos [i].y = 10;
    }
    for (int i = 0; i < obs_num; i++) {
        targ_pos [i].x = 10;
        targ_pos [i].y = 10;
    }
    char IO_msg [240], tmp_str [240];
    int obs_placed = obs_num;
    int targ_placed = targ_num;
    

    int mapsize [2];
    int kb_res;
    sigset_t select_mask, orig_mask;

    if (sigemptyset (&select_mask) < 0) {
        printerror ("mask creation 1");
    }
    if (sigaddset (&select_mask, SIGUSR1) < 0) {
        printerror ("mask creation 2");
    }
    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (fd_in_server, &fd_input);
    printf ("waiting for start message from blackboard on file descriptor %d\n", fd_in_server);
    if (pselect (fd_in_server + 1, &fd_input, NULL, NULL, NULL, &select_mask) < 0) {
        printerror ("start wait select");
    }
    if (read (fd_in_server, IO_msg, 160) < 0) {
        printerror ("wait read");
    }
    //printf ("start message received, starting\n");
    long int time_to_sleep = 20;
    /*do {
        time_to_sleep = sleep (time_to_sleep);
    } while (time_to_sleep > 0);*/

    
    
    initscr(); 

    

    struct timespec t;
    struct timespec end_time;
    struct timespec delta_time, rem_time;
    //long int time_to_sleep;
    long int nsec_diff;
    void * memcopy_res = NULL;
    int reset = 0;
    
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
        init_pair (4, COLOR_GREEN, COLOR_BLACK);

    }

    
    

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
            //read to wait for server acknowledgement (else risk of permanent block on other requests' read)
            if (read (fd_in_server, IO_msg, 160) < 0) {
                printerror ("BB reset response read");
            }
            curr_targ = 0;
        }

        /*update of the last first taret not yet reached*/
        //printw ("requesting target to reach\n\r");
        //refresh ();
        request_current_target (out_fd_server, fd_in_server, &curr_targ, IO_msg, &reset, &select_mask);
        //printw ("targget request  completed\n\r");
        //refresh ();
        //printerror ("test block 1");
        if (curr_targ >= targ_placed) { //all targets reached, needed reset
            if (sprintf (IO_msg, "%c", 'r') < 0) {
                printerror ("sprintf");
            }
            if (write (out_fd_server, IO_msg, strlen (IO_msg) + 1) < 0) {
                printerror ("write");
            }
            if (sprintf (IO_msg, "%d", 'z') < 0) 
                printerror ("reset drone message encoding");
            if (write (fd_drone, IO_msg, strlen (IO_msg) + 1) < 0) 
                printerror ("reset drone write");

            //read to wait for server acknowledgement (else risk of permanent block on other requests' read)
            do {
                if (syscall_res = read (fd_in_server, IO_msg, 160) < 0) {
                    if (errno == EINTR) continue;
                    printerror ("BB reset response read");
                }
            } while (syscall_res < 0);
        
            curr_targ = 0;
            targ_placed = targ_num;
        }
        
        

        //printw ("requesting drobbne position\n\r");
        //refresh();
        read_drone_from_BB (fd_in_server, out_fd_server, &drone_pos, IO_msg, &t, &select_mask);
        //printw ("requesting obsatcle position\n\r");
        //refresh ();
        request_obs (fd_in_server, out_fd_server, &obs_placed, obs_pos, IO_msg, tmp_str, &t, &select_mask);
        //printw ("requesting target position\n\r");
        //refresh ();
        request_targ (fd_in_server, out_fd_server, &targ_placed, targ_pos, IO_msg, tmp_str, &t, &select_mask);
        //printing stuff on terminal, blocking SIGUSR1 to avoid errors in the functions
        if (sigprocmask (SIG_BLOCK, &select_mask, &orig_mask) < 0) {
            printerror ("mask setting 1");
        }
        syscall_res = clear ();
        if (syscall_res == ERR) {
            printf ("issues clearing\n\r");
            printerror ("addch clear");
        }
        syscall_res = wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE); //makes sure the playground (visualized using box () below) is of the expected size
        if (syscall_res == ERR) {
            printf ("issues resizing\n\r");
            printerror ("resizing");
        }/**/
        syscall_res = resize_term (MAP_Y_SIZE, MAP_X_SIZE);
        if (syscall_res == ERR) {
            printerror ("terminal resize");
        }
        //printw ("all values requested\n\r");
        //refresh ();
        //set character color to white for border, the re-set it back to default
        

        attron (COLOR_PAIR (2));
        box (stdscr, '|', '-');
        attroff (COLOR_PAIR (2));

        attron (COLOR_PAIR (1));
        while (mvprintw ((int) round (drone_pos.y), (int) round (drone_pos.x), "%c", DRONE_MARKER) < 0) {
        //do {
        //syscall_res = printw ("%lf %lf\n\r", drone_pos.x, drone_pos.y);
        //if (syscall_res == ERR) {
        /*    sprintf (IO_msg, "%lf %lf %s", drone_pos.x, drone_pos.y, "drone placement");
            printerror (IO_msg);
        }/**/
        //} while (syscall_res < 0);
        }
        //refresh ();
        attroff (COLOR_PAIR (1));
        attron (COLOR_PAIR (3)); 
        //syscall_res = 10; 
        /*if (mvprintw (MAP_Y_SIZE/2, MAP_X_SIZE/2, "%d\n", obs_placed) < 0)
            printerror ("obs test 1\n"); 
        for (int i = 0; i < obs_placed; i++) {
            if (printw ("%lf %lf\n", obs_pos [i].x, MAP_Y_SIZE - obs_pos [i].y) < 0) {
                sprintf (IO_msg, "%d %s", i ,"obs test 2");
                printerror (IO_msg);
            }
        }
        refresh ();
        //sigprocmask (SIG_SETMASK, &orig_mask, NULL);
        time_to_sleep = sleep (10);
        while (time_to_sleep > 0) {
            time_to_sleep = sleep (time_to_sleep);
        }
        //sigprocmask (SIG_SETMASK, &select_mask, &orig_mask);
        */
        for (int i = 0; i < obs_placed; i++) {
            double x_pos = obs_pos [i].x;
            double y_pos = obs_pos [i].y;
            
            //while (printw ("%lf %lf\n\r", obs_pos [i].x, obs_pos [i].y) < 0) {
            while (mvprintw ((int) round (y_pos), (int) round (x_pos), "%c", OBSTACLE_MARKER) < 0) {
            //if (syscall_res == ERR) { 
                //printf ("%d\n", i);
                /*sprintf (IO_msg, "%lf %lf, %d %s", obs_pos[i].y, y_pos, i, "obstacle placement");
                //printf ("%lf %lf\n", obs_pos [i].y, y_pos);
                printerror (IO_msg);/**/
            }
        }
        //refresh ();
        
        attroff (COLOR_PAIR (3));

        attron (COLOR_PAIR (4));
        //if (mvprintw (MAP_Y_SIZE/2, MAP_X_SIZE/2, "%d\n", targ_placed) < 0) {
        for (int i = curr_targ; i < targ_placed; i++) {
            while (mvprintw (round (targ_pos [i].y), round (targ_pos [i].x), "%s", Targ_marker [i]) < 0) {
            //printw ("%lf %lf\n", targ_pos [i].x, MAP_Y_SIZE - targ_pos[i].y);
                //printerror ("targget placement");
            }
        }
        /*if (syscall_res < 0) 
            printerror ("targget placement");
        //attroff (COLOR_PAIR (4));/**/


        
        refresh ();
        if (sigprocmask(SIG_UNBLOCK, &select_mask, NULL) < 0) {
            printerror ("mask unsetting");
        }

        //count = (count + 1) % maxcount;

        usleep (SEC_TO_USEC / framerate);


    }
    return 0;
}