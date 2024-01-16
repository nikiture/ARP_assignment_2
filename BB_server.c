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

const int pipe_numb = 3; //dynamics, kb + map, obstacles, (targets)
const int log_id = 2; //position in log_file where pid is written
const int frequency = 200;
int time_left = 5;
int reset = 0;
int quit = 0;

int process_request (int, int, double [2], double [2] [obs_num], double [2] [targ_num], int [pipe_numb], char [240], char [240], int *, int *);

void reset_obstacles (int obs_fd) {
    char res_msg [2];
    if (sprintf (res_msg, "%c", 'r') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (write (obs_fd, res_msg, strlen (res_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    reset = 0;
}
void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {
        //printf ("BB: responding to server\n");
        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            printf ("BB: issues opening file\n");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
        }

        int pid = getpid ();
        if (pid < 0) {
            perror ("getpid");
            printf ("BB: issues getting pid");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
        }
        //printf ("%d", pid);
        char logdata [10];
        
        if (sprintf (logdata, "%c", 'r') < 0) {
            perror ("sprintf");
            printf ("BB: issues formatting\n");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
        }

        if (write (fd, logdata, sizeof (logdata)) < 0) {
            perror ("write");
            printf ("BB: issues writing to file\n");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
        }

        //close (fd);
        //printf ("BB: finished responding\n");
    }
}

void send_quit_message (int fd_obs) {
    char quit_msg [2];
    if (sprintf (quit_msg, "%c", 'q') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (write (fd_obs, quit_msg, strlen (quit_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
}


int main (int argc, char* argv[] ) {
    printf ("%d\n", getpid());
    
    //signal (SIGUSR1, &watchdog_req);
    /*struct sigaction watchdog_sig;
    memset (&watchdog_sig, 0, sizeof (watchdog_sig));
    watchdog_sig.sa_handler = &watchdog_req;*/

    /*sigset_t select_mask;
    sigaddset (&select_mask, SIGUSR1);*/
    //sigaction (SIGUSR1, &watchdog_sig, NULL);

    signal (SIGUSR1, watchdog_req);
    //for now the server does not store the keyboard input, that is exchanged directly between dynamics process and keyboard input process
    srand (time (NULL));
    //printf ("%d\n", rand () % 10);
    //server write in pipe to watchdog its pid
    printf ("hello from the blackboard\n");
    //printf ("BB: pid: %d\n", getpid());

    int fd = open (log_file [log_id], O_WRONLY);
    if (fd < 0) {
        perror ("log_file open");
        printf ("issues opening file\n");
    }
    printf ("blacboard: opened log file\n");

    int pid = getpid ();
    if (pid < 0) {
        perror ("pid reception");
        exit (EXIT_FAILURE);
    }
    //printf ("%d", pid);
    char logdata [10];
    
    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("sprintf");
        printf ("issues formatting\n");
    }
    

    if (write (fd, logdata, sizeof (logdata)) < 0) {
        perror ("write");
        printf ("issues writing to file\n");
    }
    printf ("blacboard: written to watchdog pid\n");

    if (close (fd) < 0) {
        perror ("close");
    }
    
    short int rand_reader;
    int fd_in [pipe_numb]; //0 for dynamics, 1 for map + kb, 2 for obstacles, 3 target when implemented; first column for requests, second column for server's answer
    int fd_out [pipe_numb];
    
    //int BlackBoard [2] = {0, 0}; // where the values from writers and for readers are stored
    double drone_pos [2];
    drone_pos [0] = 0;
    drone_pos [1] = 0;
    double obstacle_pos [2][obs_num];
    int obs_changed [2]; //value used to store if the last obstacle position has been read by dynamics and map processes
    obs_changed [0] = obs_changed [1] = 0;
    double targ_pos [2] [targ_num];
    //int ChangedValue [2] = {0, 0}; //each cell has value 0 if the corresponding blackboard cell hasn't changed, else 1
    fd_set req_pipe;
    FD_ZERO (&req_pipe);
    struct timespec tv_w, tv_r;
    //struct timeval tv_w;

    char * single_fd_format = "%d %d";
    //char * server_format [] = {"%d %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d"};
    /*char * server_format [in_pipe_numb + out_pipe_numb]; 
    for (int i = 0; i < in_pipe_numb; i++) {
        server_format [i] 
    }*/
    //one term of argv per pipe needed
    printf ("BB: deformatting pipe fds\n");
    if (argc < pipe_numb + 1) {
        printf ("server: not enough input arguments!\n");
        exit (EXIT_FAILURE);
    }
    
    int read_req, write_req;
    int wait_for_req [pipe_numb];//array of values to check if the corresponding processes are expected to send a request or actual data to server
    
    for (int i = 0; i < pipe_numb; i++) {
        wait_for_req [i] = 1;
    }
    int rand_number;
    //biggest fd between the input ones, used for select () 
    
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
    
    int maxfd = -10;
    for (int i = 0; i < pipe_numb; i++) {
        if (maxfd < fd_in [i]) maxfd = fd_in [i];
    }
    maxfd++;
    for (int i = 0; i < pipe_numb; i++) {
        printf ("%d %d\n", fd_in [i], fd_out [i]);
    }
    printf ("finished deformatting pipes\n");
    //printf ("%d", fd_in [0] [0]);
    //the server needs to read (in end) from asking pipes and write (out end) on answer pipes;
    //here it closes the unused ends of the pipes
    /*for (int i = 0; i < pipe_numb; i++) {
        close (fd_in [i] [1]);
        close (fd_out [i] [0]);
    }*/


    //char rand_value_string [80];
    //printf ("here");
    
    //printf ("%d %d", fd_in [0] [0], fd_out [0] [1]);
    //rand_reader = open ("/dev/random", O_RDONLY);
    int read_res;
    int simul_req [pipe_numb]; //stores at each loop which pipes have data to be read (used for fair undeterministic selection)
    char request_string [240], answer_string [240], start_str [240];
    /*if (read (fd_in [1], request_string, 240) < 0) {
        perror ("read test map");
        exit (EXIT_FAILURE);
    }
    printf ("the connection with the map seems to be working\n");*/
    //sprintf (start_str, "%s", "s");
    int req_idx, input_value, sel_idx;
    int obs_placed;
    //printf ("debug 2\n"); 
    //printf ("sending starting message to processes");
    /*for (int i = 0; i < pipe_numb; i++) {
        write (fd_out [i] [1], start_str, strlen (start_str) + 1);
    }*/
    //mask to block SIGUSR1 while in select
    sigset_t select_mask;
    sigaddset (&select_mask, SIGUSR1);
    int count = 0;

    //writes to every process to "let them know" it's ready and to make them start 
    if (sprintf (answer_string, "%c", 'r') < 0) {
        perror ("ready sprintf");
        exit (EXIT_FAILURE);
    }
    for (int i = 0; i < pipe_numb; i++) {
        if (write (fd_out [i], answer_string, strlen (answer_string)) < 0) {
            perror ("ready write");
            exit (EXIT_FAILURE);
        }
    }
    //waits for the obstacles to be generated and  sent before working fairly on all processes (to ensure proper initialisation for both dynamics and map displayer processes)
    if (read (fd_in [2], request_string, 160) < 0) {
        perror ("obstacles read");
        exit (EXIT_FAILURE);
    }
    if (process_request (2, fd_out [2], drone_pos, obstacle_pos, targ_pos, wait_for_req, request_string, answer_string, obs_changed, &obs_placed) < 0) {
        perror ("obstacle pre-loop processing");
        exit (EXIT_FAILURE);
    }

    while (1) {
        /*read (fd_in [1], request_string, 240);
        process_request (1, fd_out [1], drone_pos, obstacle_pos, targ_pos, wait_for_req, request_string, answer_string, obs_changed, &obs_placed);
        continue;*/
        tv_w.tv_sec = 0;
        tv_w.tv_nsec = 1000;
        FD_ZERO (&req_pipe);
        //FD_ZERO (&write_pipe);
        //printf ("setting fd_set\n");
        for (int i = 0; i < pipe_numb; i++) {
            FD_SET (fd_in [i], &req_pipe);
            //FD_SET (fd_in [i + 2], &read_pipe);
        }
        //printf ("here1\n");
         //how many obstacles are actually placed on the map
        //printf ("looking for requests from processes\n");

        write_req = pselect (1024, &req_pipe, NULL, NULL, &tv_w, &select_mask);
        //printf ("finished looking up, start selection of requester to process\n");
        //printf ("request from %d process\n", write_req);
        //write_req = select (maxfd + 1, &req_pipe, NULL, NULL, &tv_w);
        
        if (write_req < 0) {//error in select ()
            perror ("select");
            printf ("issues with select\n");
            do {
                time_left = sleep (time_left);
            } while (time_left > 0);
            close (rand_reader);
            for (int i = 0; i < pipe_numb; i++){
                close (fd_in [i]);
                close (fd_out [i]);
            } 
            //perror ("select");
            exit (EXIT_FAILURE);
        }
        //printf ("completed error checking on select\n");
        //printf ("%d\n", write_req);
        switch (write_req) {
            case 0://no requests sent to server
                printf ("no request received\n");
                //continue;
                break;
            case 1: //only one request: no need to randomly choose but need to identify requester for proper action
                for (req_idx = 0; req_idx < pipe_numb; req_idx ++) {
                    if (FD_ISSET (fd_in [req_idx], &req_pipe)) break;
                }
                syscall_res = read (fd_in [req_idx], request_string, sizeof (request_string));
                if (syscall_res < 0) {
                    perror ("read");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    for (int i = 0; i < pipe_numb; i++) {
                        close (fd_in [i]);
                        close (fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                //printf ("starting processing of request\n");
                if (process_request (req_idx, fd_out [req_idx], drone_pos, obstacle_pos, targ_pos, wait_for_req, request_string, answer_string, obs_changed, &obs_placed) < 0) {
                    perror ("process requets");
                    printf ("issues processing request, bye");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    exit(EXIT_FAILURE);
                }
                //printf ("finished processing of request\n");
                break;        
            default: //more than one process has pending request, random choice for fairness after identifying requesting processes
                //printf ("more than one process requesting, starting random selection\n");
                req_idx = 0;
                for (int i = 0; i < pipe_numb ; i++) {//loop to store indexes (each one corresponding to a different process) in request queues
                    if (FD_ISSET (fd_in [i], &req_pipe)) {
                        simul_req [req_idx] = i;
                        req_idx++;
                    }
                }
                //printf ("picking random number\n");
                //take random index between the ones which have a request to process
                rand_selector = rand ();
                //printf ("%d\n", rand_selector);
                if (rand < 0) {
                    perror ("rand");
                    exit (EXIT_FAILURE);
                }
                sel_idx = rand_selector % write_req;
                req_idx = simul_req [sel_idx];
                //printf ("%d\n", sel_idx);
                syscall_res = read (fd_in [req_idx], request_string, 240);
                if (syscall_res < 0) {
                    perror ("read");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    for (int i = 0; i < pipe_numb; i++) {
                        close (fd_in [i]);
                        close (fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                //printf ("starting processing of request\n");
                if (process_request (req_idx, fd_out [req_idx], drone_pos, obstacle_pos, targ_pos, wait_for_req, request_string, answer_string, obs_changed, &obs_placed) < 0) {
                    perror ("process requets");
                    printf ("issues processing request, bye");
                    do {
                        time_left = sleep (time_left);
                    } while (time_left > 0);
                    exit(EXIT_FAILURE);
                }
                //printf ("finished processing of request\n");
                break;
        }
        if (reset == 1) {
            reset_obstacles (fd_out [2]);
        }
        if (quit == 1) {
            send_quit_message (fd_out [2]);
            exit (EXIT_SUCCESS);
        }
        //printf ("%lf %lf", drone_pos [0], drone_pos [1]);
        count = (count + 1) % 20;
        /*if (count == 0) {
            for (int i = 0; i < obs_placed; i++) {
                printf ("%lf %lf\n", obstacle_pos [0] [i], obstacle_pos [1] [i]);
            }
        }*/
        usleep (SEC_TO_USEC/frequency);
            //this code should be already implemented generalized in the switch
            /*if (FD_ISSET (fd_in [0], &req_pipe) && FD_ISSET(fd_in [1], &req_pipe)) { //both asking pipes from writer 0 and writer 1 are ready 
                //read a random value from /dev/rand and reduce it mod <number of reader processes> to select randomly which pipe  to read from
                if (read (rand_reader, &rand_number, sizeof (short int)) < 0) {
                    close (rand_reader);
                    for (int i = 0; i < pipe_numb; i++){
                        close (fd_in [i] [0]);
                        close (fd_out [i] [1]);
                    }  
                    perror ("rand value reading");
                    exit (EXIT_FAILURE);
                } else
                    req_idx = rand_number % 2; 
            } else if (FD_ISSET (fd_in [0] [0], &write_pipe)) { 
                idx = 0;
            }
            else { 
                idx = 1;
            }*/
        //this section should be revisited and moved in process_request
        /*syscall_res = read (fd_in [req_idx], request_string, sizeof ("r"));
            //printf ("%s", request_string);
            //printf ("request from writer %d received\n", idx);
            //read (fd_in [idx] [0], request_string, 80);
            //sprintf (request_string, "%d", input_value);
            if ((BlackBoard [0] == 0 && BlackBoard [1] == 0) || BlackBoard[1] <= BlackBoard [1 - 1]) {
                
                
                if (write (fd_out [req_idx] [1], "y", sizeof ("y")) < 0) {
                    close (rand_reader);
                    for (int i = 0; i < pipe_numb; i++){
                        close (fd_in [i]);
                        close (fd_out [i]);
                    } 
                    perror ("write");
                    //sleep (6);
                    exit (EXIT_FAILURE);
                }
                
                printf ("request approved! waiting to receive the value\n");
                usleep (10);
                read (fd_in [idx] [0], request_string, sizeof (int) + 1);

                sscanf (request_string, "%d", &input_value);

                printf ("Storing value %d in cell %d\n", input_value, idx);
                BlackBoard [idx] = input_value;
                ChangedValue [idx] = 1;
                //sprintf (answer_string, "y");
                //write (fd_out [idx] [1], answer_string, strlen (answer_string) + 1);
            } else {
                printf ("request denied\n");
                sprintf (answer_string, "n");
                write (fd_out [idx] [1], answer_string, strlen (answer_string) + 1);
            }*/
        //}
        //this part should be included in the previous one
        
        /*tv_r.tv_sec = 1;
        tv_r.tv_usec = 0;
        read_req = select (1024, &read_pipe, NULL, NULL, &tv_r);
        if (read_req == -1) { // error in select()
            close (rand_reader);
            for (int i = 0; i < pipe_numb; i++){
                close (fd_in [i] [0]);
                close (fd_out [i] [1]);
            } 
            perror ("select");
            exit (EXIT_FAILURE);
        } else if (read_req != 0) { // at least a reader requests reading
            if (FD_ISSET (fd_in [2] [0], &read_pipe) && FD_ISSET(fd_in [3] [0], &read_pipe)) { //both asking pipes from reader 0 and reader 1 are ready 
                //read a random value from /dev/rand and reduce it mod <number of reader processes> to select randomly which pipe  to read from
                if (read (rand_reader, &rand_number, sizeof (short int)) <0) {
                    close (rand_reader);
                    for (int i = 0; i<pipe_numb; i++){
                        close (fd_in [i] [0]);
                        close (fd_out [i] [1]);
                    }  
                    perror ("rand value read");
                    exit (EXIT_FAILURE);
                }
                idx = 2 + rand_number%2; 
            } else if (FD_ISSET (fd_in [2] [0], &read_pipe)) { 
                idx = 2;
            }
            else {
                idx = 3;
            } read (fd_in [idx] [0], request_string, 80);
            printf ("request from reader %d received and ", idx - 2);
            if (ChangedValue [idx]) { // corresponding value has changed, therefore the server accepts the request from reader idx
                sprintf (answer_string, "y");
                write (fd_out [idx] [1], answer_string, strlen (answer_string) + 1);
                sprintf (answer_string, "%d", BlackBoard [idx]);
                write (fd_out [idx] [1], answer_string, strlen (answer_string) + 1);
                printf ("approved.\n");
            } else { // value hasn't changed: reading request to be denied
                sprintf (answer_string, "n");
                printf ("denied.\n");
                write (fd_out [idx] [1], answer_string, strlen (answer_string) + 1);
            } 
        } else {
            printf ("no read request received\n");
        }*/
        //sleep (1);


    }
    return 0;
}

int process_request (int req_idx, int request_fd_o, double drone_pos [2], double obs_pos [2] [obs_num], double targ_pos [2] [targ_num], int wait_for_request [pipe_numb], char pipe_msg [240], char out_msg [240], int * changed_obstacles, int * obs_placed) {
    char tmp_msg [240];
    switch (req_idx) {
        case 0: //this is drone dynamic: it can send the updated drone position or request the obtacle position
            printf ("processing request from dynamics process\n");
            if (pipe_msg [0] != 'r') { //the process is writing the updated drone position
                //printf ("receiving drone position\n");
                if (sscanf (pipe_msg, "%lf %lf", &drone_pos [0], &drone_pos [1]) < 0) {
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
                //printf ("%lf %lf\n", drone_pos [0], drone_pos [1]);
            } else { //the process is making a request, sending obstacle position if they have changed
                //printf ("obstacle request from dynamics\n");
                if (changed_obstacles [0] == 1) {
                    //printf ("request approved\n");
                    if (sprintf (out_msg, "%c%d%c", '[', *obs_placed, ']') < 0){
                        perror ("message formation 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);       
                        return -1;    
                    }
                    for (int i = 0; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [0] [i], ',', obs_pos [1] [i]) < 0) {
                            perror ("message formation 2");
                            do {
                                time_left = sleep (time_left);
                            } while (time_left > 0);
                            return -1;
                        }
                        if (strcpy (out_msg, tmp_msg) < 0) {
                            perror ("message formation 3");
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
                    //printf ("request denied\n");
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
            //printf ("completed response to drone process\n");
            break;
        case 1: //map displayer: can request drone or obstacle positions or write to signal a reset
            printf ("processing request from map process\n");
            if (pipe_msg [0] == 'd') {//requested drone position
            //printf ("sending to map process drone position\n");
                if (sprintf (out_msg, "%.3lf %.3lf", drone_pos [0], drone_pos [1]) < 0) {
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
                /*printf ("values: %lf3 %lf3\n", drone_pos [0], drone_pos [1]);
                printf ("string: %s\n", out_msg);*/
            } else if (pipe_msg [0] == 'z') {
                printf ("received from map process reset instruction, sending to obstacle generator soon\n");
                //server needs to send reset message to obstacle process
                reset = 1;
                if (sprintf (out_msg, "%c", 'o') < 0) {
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
            } else if (pipe_msg [0] == 'q') {//quit message 
                quit = 1;
            }
            else {//requested obstacles position
            //printf ("sending to map process obstacles position\n");
                if (changed_obstacles [1] == 1) {
                    if (sprintf (out_msg, "%c%d%c%.3lf%c%.3lf", '[', *obs_placed, ']', obs_pos [0] [0], ',', obs_pos [1] [0]) < 0) {
                        perror ("map message formation obs 1");
                        do {
                            time_left = sleep (time_left);
                        } while (time_left > 0);
                        return -1;
                    }
                    for (int i = 1; i < *obs_placed; i++) {
                        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [0] [i], ',', obs_pos [1] [i]) < 0) {
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
                    changed_obstacles [1] = 0;
                } else {
                    if (sprintf (out_msg, "%c", 'n') < 0) {
                        perror ("map formating");
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
            }
            //printf ("written response to map\n");
            //printf ("%s\n", out_msg);
            break;
        case 2: //obstacle process: sends updated positions of obstacles
            printf ("processing obstacles\n");
            //printf ("%s", pipe_msg);
            //char * char_dump;
            changed_obstacles [0] = changed_obstacles [1] = 1;
            //printf ("%s\n", pipe_msg); //strings are the same, issues in formatting
            sscanf (pipe_msg, "%*c%d%*c%lf%*c%lf%s", obs_placed, &obs_pos [1] [0], &obs_pos [0] [0], tmp_msg); //in pipe_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
            //printf ("%s\n", tmp_msg);
            /*strcpy (pipe_msg, tmp_msg);
            if (*obs_placed > obs_num) *obs_placed = obs_num;
            sscanf (pipe_msg, "%lf%lf%s", &obs_pos [0] [0], &obs_pos [1] [0], tmp_msg);*/
            strcpy (pipe_msg, tmp_msg);
            //printf ("%lf %lf\n", obs_pos [0] [0], obs_pos [1] [0]);
            for (int i = 1; i < *obs_placed; i++) {
                sscanf (pipe_msg, "%*c%lf%*c%lf%s", &obs_pos [1] [i], &obs_pos [0] [i], tmp_msg);
                strcpy (pipe_msg, tmp_msg);
            }
            break;
        default: //unexpexted value for process
            //printf ("process identifier not recognized\n");
            break;
        
    }

    return 1;
}

