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

const int pipe_numb = 4; //dynamics, kb + map, obstacles, (targets)
const int log_id = 2; //position in log_file where pid is written
const int frequency = 200;
int time_left = 5;
int reset = 0;
int quit = 0;

int process_request (int, int, struct position *, struct position [obs_num], struct position [targ_num], char [240], char [240], int *, int *, int *, int *, int *);

void reset_obstacles_targets (int obs_fd, int targ_fd) {
    char res_msg [2];
    if (sprintf (res_msg, "%c", 'r') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (write (obs_fd, res_msg, strlen (res_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    if (write (targ_fd, res_msg, strlen (res_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    reset = 0;
}
void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {

        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
            exit (EXIT_FAILURE);
        }

        int pid = getpid ();
        if (pid < 0) {
            perror ("getpid");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
            exit (EXIT_FAILURE);
        }

        char logdata [10];
        
        if (sprintf (logdata, "%c", 'r') < 0) {
            perror ("sprintf");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
            exit (EXIT_FAILURE);
        }

        if (write (fd, logdata, sizeof (logdata)) < 0) {
            perror ("write");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
            exit (EXIT_FAILURE);
        }


    }
}

void send_quit_message (int fd_obs, int fd_targs) {
    char quit_msg [2];
    if (sprintf (quit_msg, "%c", 'q') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (write (fd_obs, quit_msg, strlen (quit_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    if (write (fd_targs, quit_msg, strlen (quit_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
}


int main (int argc, char* argv[] ) {
    

    signal (SIGUSR1, watchdog_req);
    //The server does not store the keyboard input, that is exchanged directly between dynamics process and keyboard input process
    srand (time (NULL));
    //server write in pipe to watchdog its pid (so that the watchdog knows what processes to send signals to)

    int fd = open (log_file [log_id], O_WRONLY);
    if (fd < 0) {
        perror ("log_file open");
        exit (EXIT_FAILURE);
    }

    int pid = getpid ();
    if (pid < 0) {
        perror ("pid reception");
        exit (EXIT_FAILURE);
    }

    char logdata [10];
    
    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    

    if (write (fd, logdata, sizeof (logdata)) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }

    if (close (fd) < 0) {
        perror ("close");
        exit (EXIT_FAILURE);
    }
    
    short int rand_reader;
    int fd_in [pipe_numb]; //0 for dynamics, 1 for map + kb, 2 for obstacles, 3 target when implemented; first column for requests, second column for server's answer
    int fd_out [pipe_numb];
    
    //double drone_pos [2];
    struct position drone_pos;
    drone_pos.x = 0;
    drone_pos.y = 0;
    struct position obstacle_pos [obs_num];
    int obs_changed [2]; //value used to store if the last obstacle position has been read by dynamics and map processes
    obs_changed [0] = obs_changed [1] = 0;
    struct position targ_pos [targ_num];
    int targ_changed [2];
    targ_changed [0] = targ_changed [1] = 0;
    int targ_placed;
    int targ_to_reach = 0;
    //double targ_pos [2] [targ_num];
    
    fd_set req_pipe;
    FD_ZERO (&req_pipe);
    struct timespec tv_w;

    
    //one term of argv per pipe needed

    if (argc < pipe_numb + 1) {
        printf ("server: not enough input arguments!\n");
        exit (EXIT_FAILURE);
    }
    
    int read_req, write_req;
    
    
    int rand_number;
    
    
    //printf ("here1 \n %d\n", argc);
    //printf ("%s", argv [0]);
    //sscanf (argv[1], server_format, &fd_in [0] [0], &fd_in [0] [1], &fd_in [1] [0], &fd_in [1] [1], &fd_in [2] [0], &fd_in [2] [1], &fd_in [3] [0], &fd_in [3] [1], &fd_out [0] [0], &fd_out [0] [1], &fd_out [1] [0], &fd_out [1] [1], &fd_out [2] [0], &fd_out [2] [1], &fd_out [3] [0], &fd_out [3] [1]);
    //sscanf (fd_str, server_format, fd [0], fd [1], fd [2], fd [3]);

    int syscall_res, rand_selector;

    for (int i = 0; i < pipe_numb; i ++) {
        syscall_res = sscanf (argv [i + 1], "%d %d", &fd_in [i], &fd_out [i]);
        if (syscall_res < 0) {
            perror ("server: args sscanf");
            exit (EXIT_FAILURE);
        }
    }
    //biggest fd between the input ones + 1, used for select () 
    int maxfd = -10;
    for (int i = 0; i < pipe_numb; i++) {
        if (maxfd < fd_in [i]) maxfd = fd_in [i];
    }
    maxfd++;
    
    int read_res;
    int simul_req [pipe_numb]; //stores at each loop which pipes have data to be read (used for fair undeterministic selection)
    char request_string [240], answer_string [240], start_str [240];
    
    int req_idx, sel_idx;
    int obs_placed; //variable used to store the actual number of obstacle placed, to be used particularly for assignment 3 or if implemented generator process with random amount of obstacle each time
    
    //mask to block SIGUSR1 while in select
    sigset_t select_mask;
    sigemptyset (&select_mask);
    sigaddset (&select_mask, SIGUSR1);

    
    //waits for the obstacles to be generated, sent and stored before entering the continuous loop and working fairly on all processes (to ensure proper initialisation for both dynamics and map displayer processes)
    if (read (fd_in [2], request_string, 240) < 0) {
        perror ("obstacles read");
        exit (EXIT_FAILURE);
    }
    if (process_request (2, fd_out [2], &drone_pos, obstacle_pos, targ_pos, request_string, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
        perror ("obstacle pre-loop processing");
        exit (EXIT_FAILURE);
    }
    if (read (fd_in [3], request_string, 240) < 0) {
        perror ("target read");
        exit (EXIT_FAILURE);
    }
    if (process_request (3, fd_out [3], &drone_pos, obstacle_pos, targ_pos, request_string, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
        perror ("target pre-loop processing");
        exit (EXIT_FAILURE);
    }

    //writes to every process but the generators and the watchdog to "let them know" it's ready and to make them start 
    if (sprintf (answer_string, "%c", 'r') < 0) {
        perror ("ready sprintf");
        exit (EXIT_FAILURE);
    }
    for (int i = 0; i < pipe_numb; i++) {
        if (i == 2 || i == 3) continue;
        if (write (fd_out [i], answer_string, strlen (answer_string) + 1) < 0) {
            perror ("ready write");
            exit (EXIT_FAILURE);
        }
        printf ("ready message successfully sent to process %d\n", i);
    }

    while (1) {
        
        tv_w.tv_sec = 0;
        tv_w.tv_nsec = 1000 * SEC_TO_USEC / frequency;
        FD_ZERO (&req_pipe);

        for (int i = 0; i < pipe_numb; i++) {
            FD_SET (fd_in [i], &req_pipe);

        }


        write_req = pselect (1024, &req_pipe, NULL, NULL, &tv_w, &select_mask);
        
        
        if (write_req < 0) {//error in select ()
            perror ("select");
            close (rand_reader);
            for (int i = 0; i < pipe_numb; i++){
                close (fd_in [i]);
                close (fd_out [i]);
            } 

            exit (EXIT_FAILURE);
        }

        switch (write_req) {
            case 0://no requests sent to server
                break;
            case 1: //only one request: no need to randomly choose but need to identify requester (its id) for proper action (fd- and data transmission-wise)
                //find which process has written
                for (req_idx = 0; req_idx < pipe_numb; req_idx ++) {
                    if (FD_ISSET (fd_in [req_idx], &req_pipe)) break;
                }
                //read message from pipe
                syscall_res = read (fd_in [req_idx], request_string, sizeof (request_string));
                if (syscall_res < 0) {
                    perror ("read");
                    
                    for (int i = 0; i < pipe_numb; i++) {
                        close (fd_in [i]);
                        close (fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                //execute adequate response to message received based on content of the message and which process wrote
                if (process_request (req_idx, fd_out [req_idx], &drone_pos, obstacle_pos, targ_pos, request_string, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
                    perror ("process requets");
                    exit(EXIT_FAILURE);
                }
                break;        
            default: //more than one process has pending request, random choice for fairness after identifying requesting processes
                req_idx = 0;
                for (int i = 0; i < pipe_numb ; i++) {//loop to store indexes (each one corresponding to a different process) in request queues
                    if (FD_ISSET (fd_in [i], &req_pipe)) {
                        simul_req [req_idx] = i;
                        req_idx++;
                    }
                }
                //take random index between the ones which have a request to process
                rand_selector = rand ();
                if (rand < 0) {
                    perror ("rand");
                    exit (EXIT_FAILURE);
                }
                sel_idx = rand_selector % write_req;
                req_idx = simul_req [sel_idx];

                syscall_res = read (fd_in [req_idx], request_string, 240);
                if (syscall_res < 0) {
                    perror ("read");
                    
                    for (int i = 0; i < pipe_numb; i++) {
                        close (fd_in [i]);
                        close (fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }

                if (process_request (req_idx, fd_out [req_idx], &drone_pos, obstacle_pos, targ_pos, request_string, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
                    perror ("process requets");
                    
                    exit(EXIT_FAILURE);
                }
                break;
        }
        if (reset == 1) { //received reset instruction from the keyboard process

            reset_obstacles_targets (fd_out [2], fd_out [3]);
            printf ("sending reset message to the generating processes\n");
            targ_to_reach = 0;
            reset = 0;
            
        }
        if (quit == 1) { //received quit instruction from the keyboard process
            send_quit_message (fd_out [2], fd_out [3]);
            exit (EXIT_SUCCESS);
        }

        //usleep (SEC_TO_USEC/frequency);

    }
    return 0;
}

int process_request (int req_idx, int request_fd_o, struct position * drone_pos, struct position obs_pos [obs_num], struct position targ_pos [targ_num], char pipe_msg [240], char out_msg [240], int * changed_obstacles, int * obs_placed, int changed_targets [2], int * targ_placed, int * curr_targ) {
    char tmp_msg [240];
    switch (req_idx) {
        case 0: //this is drone dynamic: it can send the updated drone position or request the obtacle position
            //printf ("processing request of drone process\n");
            if (pipe_msg [0] == 't') {//process requesting target position
                //printf ("target request\n");
                if (changed_targets [0] == 1) {
                    /*same formatting concept than for the dynamics process requesting obstacles*/
                    if (sprintf (out_msg, "%c%d%c%.3lf%c%.3lf", '[', *targ_placed, ']', targ_pos [0].x, ',', targ_pos [0].y) < 0) {
                        perror ("map message formation target 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    for (int i = 1; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', targ_pos [i].x, ',', targ_pos [i].y) < 0) {
                            perror ("map message formation target 2");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                        if (strcpy (out_msg, tmp_msg) < 0) {
                            perror ("map message formation target 3");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                    } 
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    changed_targets [0] = 0;
                } else {
                    if (sprintf (out_msg, "%c", 'n') < 0) {
                        perror ("map formating");
                        return -1;
                    }
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                }
            }            
            else if (pipe_msg [0] == 'o') { //the process is making a obstacle request, sending obstacle position if they have changed
                //printf ("obstacle request\n");
                if (changed_obstacles [0] == 1) {
                    /*formatting the obstacles position in the format "[number-of-obstacle]x_pos1,ypos1|xpos2,ypos2|..."*/
                    if (sprintf (out_msg, "%c%d%c", '[', *obs_placed, ']') < 0){
                        perror ("message formation 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);       
                        return -1;    
                    }
                    if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [0].x, ',', obs_pos [0].y) < 0) {
                        perror ("message formation 2");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    for (int i = 1; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [i].x, ',', obs_pos [i].y) < 0) {
                            perror ("message formation 3");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                        if (strcpy (out_msg, tmp_msg) < 0) {
                            perror ("message formation 4");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                    }
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("drone write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    changed_obstacles [0] = 0;
                } else { //no changes on the obstacle position, the server let the dynamics process know by writing a 'n' in the pipe
                    if (sprintf (out_msg, "%c", 'n') < 0) {
                        perror ("dorne formating");
                        return -1;
                    }
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("drone write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                }
            }
            else if (pipe_msg [0] == 'u') { //update of current target to reach
                sscanf (pipe_msg, "%*c %d", curr_targ);
                printf ("received currrent target, %d\n", *curr_targ);
                sprintf (out_msg, "%d", *curr_targ);
                write (request_fd_o, out_msg, strlen (out_msg) + 1);
            }
            else { //the process is writing the updated drone position
                //printf ("received drone position\n");
                if (sscanf (pipe_msg, "%lf %lf", &(drone_pos->x), &(drone_pos->y)) < 0) {
                    perror ("pipe scanning");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    return -1;
                }
                if (sprintf (out_msg, "%c", 'o') < 0) {
                    perror ("response message formation");
                    exit (EXIT_FAILURE);
                }
                if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                    perror ("response write");
                    exit (EXIT_FAILURE);
                }

            }
            break;
        case 1: //map displayer: can request drone or obstacle positions or write to signal a reset
            if (pipe_msg [0] == 'd') {//requested drone position
                printf ("sending drone position\n");
                if (sprintf (out_msg, "%.3lf %.3lf", drone_pos->x, drone_pos->y) < 0) {
                    perror ("map message formation drone 1");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    return -1;
                }
                if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                    perror ("display write");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    return -1;
                }
            } 
            else if (pipe_msg [0] == 'z' | pipe_msg [0] == 'r') {
                //server needs to send reset message to obstacle process
                printf ("received reset message\n");
                if (pipe_msg [0] == 'r') {
                    printf ("all target reached!\n");
                }
                reset = 1;
                if (sprintf (out_msg, "%c", 'o') < 0) {
                    perror ("map formating");
                    exit (EXIT_FAILURE);
                }
                if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                    perror ("display write");
                    exit (EXIT_FAILURE);
                }
            } 
            else if (pipe_msg [0] == 'q') {//quit message 
                quit = 1;
                //no message response needed because if at this point map process is waiting only for the watchdog to quit
            }
            else if (pipe_msg [0] == 'o') {//requested obstacles position
                printf ("received obstacle request\n");
                if (changed_obstacles [1] == 1) {
                    /*same formatting concept than for the dynamics process requesting obstacles*/
                    if (sprintf (out_msg, "%c%d%c%.3lf%c%.3lf", '[', *obs_placed, ']', obs_pos [0].x, ',', obs_pos [0].y) < 0) {
                        perror ("map message formation obs 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    for (int i = 1; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [i].x, ',', obs_pos [i].y) < 0) {
                            perror ("map message formation obs 2");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                        if (strcpy (out_msg, tmp_msg) < 0) {
                            perror ("map message formation obs 3");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                    } 
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    changed_obstacles [1] = 0;
                } else if (pipe_msg [0] == 'c') {//request of current target to reach
                    sprintf (out_msg, "%d", *curr_targ);
                    write (request_fd_o, out_msg, strlen (out_msg) + 1);
                } 
                else {
                    if (sprintf (out_msg, "%c", 'n') < 0) {
                        perror ("map formating");
                        return -1;
                    }
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                }
                
            }
            else if (pipe_msg [0] == 'c') { //request of updated current target to reach
                printf ("the current target to reach is the number %d\n", *curr_targ);
                /*if (sprintf (out_msg, "%d", *curr_targ) < 0) {
                    perror ("server current target message");
                    exit (EXIT_FAILURE);
                }
                //printf ("generated message for map process with target\n");
                //printf ("%s\n", out_msg);
                if (write (request_fd_o, out_msg, strlen (out_msg + 1)) < 0) {
                    perror ("server current target write");
                    exit (EXIT_FAILURE);
                }/**/
                if (dprintf (request_fd_o, "%d\n", *curr_targ) < 0) {
                    perror ("current target response");
                    exit (EXIT_FAILURE);
                }
                //printf ("current target successfully sent to the map process\n");
            }
            else { //target position request
                printf ("received target request\n");
                if (changed_targets [1] == 1) {
                    /*same formatting concept than for the dynamics process requesting obstacles*/
                    if (sprintf (out_msg, "%c%d%c%.3lf%c%.3lf", '[', *targ_placed, ']', targ_pos [0].x, ',', targ_pos [0].y) < 0) {
                        perror ("map message formation target 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    for (int i = 1; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', targ_pos [i].x, ',', targ_pos [i].y) < 0) {
                            perror ("map message formation target 2");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                        if (strcpy (out_msg, tmp_msg) < 0) {
                            perror ("map message formation target 3");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                    } 
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    changed_targets [1] = 0;
                } else {
                    if (sprintf (out_msg, "%c", 'n') < 0) {
                        perror ("map formating");
                        return -1;
                    }
                    if (write (request_fd_o, out_msg, strlen (out_msg) + 1) < 0) {
                        perror ("display write");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                }
            }
            break;
        case 2: //obstacle process: sends updated positions of obstacles
            changed_obstacles [0] = changed_obstacles [1] = 1;
            /*string deformatting, structure of string: [number-of -obstacles]ypos1,xpos1|ypos2,xpos2|...*/
            sscanf (pipe_msg, "%*c%d%*c%lf%*c%lf%s", obs_placed, &(obs_pos [0].y), &(obs_pos [0].x), tmp_msg); //in pipe_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
            strcpy (pipe_msg, tmp_msg);
            

            for (int i = 1; i < *obs_placed; i++) {
                sscanf (pipe_msg, "%*c%lf%*c%lf%s", &(obs_pos [i].y), &(obs_pos [i].x), tmp_msg);
                strcpy (pipe_msg, tmp_msg);
            }
            /*for (int i = 0; i < *obs_placed; i++) {
                printf ("%lf %lf\n", obs_pos [i].x, obs_pos [i].y);
            }/**/
            break;
        case 3: //target generation process: only received positions of targets
            changed_targets [0] = changed_targets [1] = 1;
            /*string deformatting, structure of string: [number-of -obstacles]ypos1,xpos1|ypos2,xpos2|...*/
            sscanf (pipe_msg, "%*c%d%*c%lf%*c%lf%s", targ_placed, &(targ_pos [0].y), &(targ_pos [0].x), tmp_msg); //in pipe_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
            strcpy (pipe_msg, tmp_msg);

            for (int i = 1; i < *targ_placed; i++) {
                sscanf (pipe_msg, "%*c%lf%*c%lf%s", &(targ_pos [i].y), &(targ_pos [i].x), tmp_msg);
                strcpy (pipe_msg, tmp_msg);
            }
            /*for (int i = 0; i < *targ_placed; i++) {
                printf ("%lf %lf\n", targ_pos [i].x, targ_pos [i].y);
            }*/
            break;
        default: //unexpexted value for process, do nothing
            break;
        
    }

    return 1;
}

