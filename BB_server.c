#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h> 
#include <stdlib.h>

int process_request (int req_idx);

int main (int argc, char* argv[] ) {
//int main (char fd_str []) {
    const int pipe_numb = 3; //dynamics, kb + map, obstacles, (targets)
    short int rand_reader;
    int fd_in [pipe_numb]; //0 for dynamics, 1 for map + kb, 2 for obstacles, 3 target when implemented; first column for requests, second column for server's answer
    int fd_out [pipe_numb];
    int BlackBoard [2] = {0, 0}; // where the values from writers and for readers are stored
    double drone_pos [2];
    int ChangedValue [2] = {0, 0}; //each cell has value 0 if the corresponding blackboard cell hasn't changed, else 1
    fd_set req_pipe;
    struct timeval tv_w, tv_r;
    char * single_fd_format = "%d %d";
    //char * server_format [] = {"%d %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d"};
    /*char * server_format [in_pipe_numb + out_pipe_numb]; 
    for (int i = 0; i < in_pipe_numb; i++) {
        server_format [i] 
    }*/
    //one term of argv per pipe needed
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
    int maxfd = -10;
    for (int i = 0; i < pipe_numb; i++) {
        if (maxfd < fd_in [i]) maxfd = fd_in [i];
    }
    maxfd++;
    //printf ("here1 \n %d\n", argc);
    //printf ("%s", argv [0]);
    //sscanf (argv[1], server_format, &fd_in [0] [0], &fd_in [0] [1], &fd_in [1] [0], &fd_in [1] [1], &fd_in [2] [0], &fd_in [2] [1], &fd_in [3] [0], &fd_in [3] [1], &fd_out [0] [0], &fd_out [0] [1], &fd_out [1] [0], &fd_out [1] [1], &fd_out [2] [0], &fd_out [2] [1], &fd_out [3] [0], &fd_out [3] [1]);
    //sscanf (fd_str, server_format, fd [0], fd [1], fd [2], fd [3]);

    int syscall_res;

    for (int i = 1; i < pipe_numb; i ++) {
        syscall_res = sscanf (argv [i], "%d %d", &fd_in [i], &fd_out [i]);
        if (syscall_res < 0) {
            perror ("server: args sscanf");
            exit (EXIT_FAILURE);
        }
    }
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
    char request_string [80], answer_string [80], start_str [80];
    //sprintf (start_str, "%s", "s");
    int req_idx, input_value;
    //printf ("debug 2\n"); 
    //printf ("sending starting message to processes");
    /*for (int i = 0; i < pipe_numb; i++) {
        write (fd_out [i] [1], start_str, strlen (start_str) + 1);
    }*/
    

    while (1) {
        tv_w.tv_sec = 1;
        tv_w.tv_usec = 0;
        FD_ZERO (&req_pipe);
        //FD_ZERO (&write_pipe);
        for (int i = 0; i < pipe_numb; i++) {
            FD_SET (fd_in [i], &req_pipe);
            //FD_SET (fd_in [i + 2], &read_pipe);
        }
        //printf ("here1\n");
        write_req = select (maxfd + 1, &req_pipe, NULL, NULL, &tv_w);
        switch (write_req) {
            case -1://error in select ()
                close (rand_reader);
                for (int i = 0; i < pipe_numb; i++){
                    close (fd_in [i]);
                    close (fd_out [i]);
                } 
                perror ("select");
                exit (EXIT_FAILURE);
                break;
            case 0://no requests sent to server
                printf ("no request received\n");
                continue;
                break;
            case 1: //only one request: no need to randomly choose but need to identify requester for proper action
                for (req_idx = 0; req_idx < pipe_numb; req_idx ++) {
                    if (FD_ISSET (fd_in [req_idx], &req_pipe)) break;
                }
                process_request (req_idx);
                break;        
            default: //more than one process has pending request, random choice for fairness
                req_idx = 0;
                for (int i = 0; i < pipe_numb; i++) {//loop to store indexes (each one corresponding to a different process) in request queues
                    if (FD_ISSET (fd_in [i], &req_pipe)) {
                        simul_req [req_idx] = i;
                        req_idx++;
                        if (req_idx >= write_req) break;
                    }
                    //take random index between the ones which have a request to process
                    req_idx = simul_req [rand () % write_req];
                }
                break;
        }
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
        sleep (1);


    }
    return 0;
}

int process_request (int req_idx) {
    switch (req_idx) {
        case 0: //this is drone dynamic requesting
    }
}