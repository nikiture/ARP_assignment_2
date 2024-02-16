#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h> 
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include "world_info.h"

const int log_id = 4; //position in log_file where pid is written
const int targ_gen_cooldown = 30; //seconds passing between obstacle generation (unless z key pressed)
//double targ_pos [targ_num] [2];
struct position targ_pos [targ_num];

void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {

        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            printf ("issues opening file\n");
        }

        int pid = getpid ();
        char logdata [10];
        
        if (sprintf (logdata, "%c", 'r') < 0) {
            perror ("sprintf");
            printf ("issues formatting\n");
        }

        if (write (fd, logdata, sizeof (logdata)) < 0) {
            perror ("write");
            printf ("issues writing to file\n");
        }

        close (fd);
    }
}


void generate_obstacles () {
    for (int i = 0; i < targ_num; i++) {
        targ_pos [i].x = ((double) rand () / ((double) RAND_MAX)) * MAP_X_SIZE;
        targ_pos [i].y = ((double) rand () / ((double) RAND_MAX)) * MAP_Y_SIZE;
        printf ("%lf %lf\n", targ_pos[i].x, targ_pos[i].y);
    }
}

void send_obstacle_to_server (int fd_out, char * out_msg, char * tmp_msg) {
    if (sprintf (out_msg, "%c%d%c", '[', targ_num, ']') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (sprintf (tmp_msg, "%s%.3lf%c%.3lf", out_msg, targ_pos [0].y, ',', targ_pos [0].x) < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (strcpy (out_msg, tmp_msg) < 0) {
        perror ("strcpy");
        exit (EXIT_FAILURE);
    }
    for (int i = 1; i < targ_num; i++) {
        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', targ_pos [i].y, ',', targ_pos [i].x) < 0) {
            perror ("sprintf");
            exit (EXIT_FAILURE);
        } 
        if (strcpy (out_msg, tmp_msg) < 0) {
            perror ("strcpy");
            exit (EXIT_FAILURE);
        }
    }
    if (write (fd_out, out_msg, strlen (out_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
}


int main (int argc, char ** argv) {

    signal (SIGUSR1, watchdog_req);
    if (argc < 2) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    int in_fd, out_fd;
    if (sscanf (argv [1], "%d %d", &in_fd, &out_fd) < 0) {
        perror ("sscanf");
        exit (EXIT_FAILURE);
    }
    char out_msg [240], tmp_msg [240];
    
    int fd = open (log_file [log_id], O_WRONLY);
    if (fd < 0) {
        perror ("log_file open");
    }

    int pid = getpid ();
    if (pid < 0) {
        perror ("pid reception");
        exit (EXIT_FAILURE);
    }

    char logdata [10];
    
    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("sprintf");
    }
    

    if (write (fd, logdata, sizeof (logdata)) < 0) {
        perror ("write");

    }

    if (close (fd) < 0) {
        perror ("close");
    }
    srand (time (NULL) + 2); //the generators and the server are spawned almost simultaneously, adding some number to time to amke sure that they have different seeds
    struct timespec t; 
    int count = 0;
    int reset_msg, time_left, sec_passed;
    sigset_t select_mask;
    sigaddset (&select_mask, SIGUSR1);

    struct timeval t_start, t_curr;
    fd_set in_pipe;

    //blocking read waiting for the server to be ready
    /*if (read (in_fd, tmp_msg, 160) < 0) {
        perror ("wait read");
        exit (EXIT_FAILURE);
    }*/

    //generate obstacles position
    generate_obstacles ();
    //send obstacle position to server
    send_obstacle_to_server (out_fd, out_msg, tmp_msg);

    gettimeofday (&t_start, NULL);
    while (1) {
        t.tv_sec = 0;
        t.tv_nsec = 0;
        FD_ZERO (&in_pipe);
        FD_SET (in_fd, &in_pipe);
        reset_msg = pselect (in_fd + 1, &in_pipe, NULL, NULL, &t, &select_mask);
        if (reset_msg < 0) {
            perror ("pselect");
            exit (EXIT_FAILURE);
        }
        if (reset_msg > 0) {
            if (read (in_fd, tmp_msg, 240) < 0) {
                perror ("read");
                exit (EXIT_FAILURE);
            }
            if (tmp_msg [0] == 'q') {
                exit (EXIT_SUCCESS);
            }
            generate_obstacles ();

            send_obstacle_to_server (out_fd, out_msg, tmp_msg);
        }
        /*gettimeofday (&t_curr, NULL);
        sec_passed = t_curr.tv_sec - t_start.tv_sec;
        if (t_curr.tv_usec < t_start.tv_usec) sec_passed --;

        if (sec_passed > targ_gen_cooldown || reset_msg > 0) {
            //generate obstacles position
            generate_obstacles ();
            //send obstacle position to server
            send_obstacle_to_server (out_fd, out_msg, tmp_msg);
            //set the new last sending time
            gettimeofday (&t_start, NULL);
        }*/
         
        time_left = sleep (1);
        while (time_left > 0) {
            time_left = sleep (time_left);
        } //ensure a sleep of 1 second even if interruption due to signal
    }
    return 0;
}