#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <signal.h>
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include "world_info.h"
#include <errno.h>
#include <fenv.h>

double PROPULSION_STEP = 1;
//approximation of sqrt(2) to avoid calling the function every time it is used in the keyboard forces computation (diagonal forces)
const double root2 = 1.41421; 
const int frequency = 100;               //frequency of dynamycs computation in hertz
#define maxforceratio 10

const int log_id = 0; //identifier of log_file where pid is written

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

int Get_Kb_In (int kb_in, int * quit, int * reset, double * kb_forces_x, double * kb_forces_y) {
    
    *quit = 0;
    *reset = 0;
    
    if (kb_in > 0) {
        switch (kb_in) {
            case 'w': //upper left
                *kb_forces_x -=  PROPULSION_STEP / root2;
                *kb_forces_y += PROPULSION_STEP / root2;
                break;
            case 's': //only left
                *kb_forces_x -= PROPULSION_STEP;
                break;
            case 'x': //down left
                *kb_forces_x -= PROPULSION_STEP / root2;
                *kb_forces_y -= PROPULSION_STEP / root2;
                break;
            case 'c': //only down
                *kb_forces_y -= PROPULSION_STEP;
                break;
            case 'v': //down right
                *kb_forces_x += PROPULSION_STEP / root2;
                *kb_forces_y -= PROPULSION_STEP / root2;
                break;
            case 'f': //only right
                *kb_forces_x += PROPULSION_STEP;
                break;
            case 'r': //up right
                *kb_forces_x += PROPULSION_STEP / root2;
                *kb_forces_y += PROPULSION_STEP / root2;
                break;
            case 'e': //only up
                *kb_forces_y += PROPULSION_STEP;
                break;
            case 'd': //brake
                //set all forces to 0
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            case 'q'://exit game
                *quit = 1;
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            case 'z': //restart
                *reset = 1;
                *kb_forces_x = 0;
                *kb_forces_y = 0;
                break;
            default: //anything different from w, s, x, c, v, f, r, e, d, q or z is considered as no press
                break;
        } 
    }
    return kb_in;
}
int update_BB (struct position pos, int out_fd, int in_fd, char * IO_msg) {


    sprintf (IO_msg, "%lf3 %lf3", pos.x, pos.y);

    write (out_fd, IO_msg, strlen (IO_msg) + 1);

    read (in_fd, IO_msg, 160);

}

void request_obstacles (int fd_in, int fd_out, char * IO_msg, int * obs_placed, struct position obs_pos [obs_num]) {
    char tmp_msg [160]; //string used for the chain of multiple sscanf() used for de-formatting all the obstacle positions sent from server
    
    if (sprintf (IO_msg, "%c", 'o') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }

    if (write (fd_out, IO_msg, strlen (IO_msg)) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }

    if (read (fd_in, IO_msg, 160) < 0) {
        perror ("response read");
        exit (EXIT_FAILURE);
    }

    if (IO_msg [0] == 'n') return;

    sscanf (IO_msg, "%*c%d%*c%s", obs_placed, tmp_msg); //in IO_msg are present the chars '[' ']' already present for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    
    strcpy (IO_msg, tmp_msg);
    
    if (*obs_placed > obs_num) *obs_placed = obs_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(obs_pos [0].x), &(obs_pos [0].y), tmp_msg);//between a pair of obstacle coordinates there is a ',' which is to be ignored
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *obs_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(obs_pos [i].x), &(obs_pos [i].y), tmp_msg); //between two pairs of obstacles positions there is a '|' to be considered but discarded
        strcpy (IO_msg, tmp_msg);
    }
    
}

void request_targets (int fd_in, int fd_out, char * IO_msg, int * targ_placed, struct position targ_pos [targ_num]) {
    char tmp_msg [160]; //string used for the chain of multiple sscanf() used for de-formatting all the obstacle positions sent from server
    
    if (sprintf (IO_msg, "%c", 't') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }

    if (write (fd_out, IO_msg, strlen (IO_msg)) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }

    if (read (fd_in, IO_msg, 160) < 0) {
        perror ("response read");
        exit (EXIT_FAILURE);
    }

    if (IO_msg [0] == 'n') return;

    sscanf (IO_msg, "%*c%d%*c%s", targ_placed, tmp_msg); //in IO_msg are present the chars '[' ']' already present for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    
    strcpy (IO_msg, tmp_msg);
    
    if (*targ_placed > obs_num) *targ_placed = obs_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(targ_pos [0].x), &(targ_pos [0].y), tmp_msg);//between a pair of obstacle coordinates there is a ',' which is to be ignored
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *targ_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(targ_pos [i].x), &(targ_pos [i].y), tmp_msg); //between two pairs of obstacles positions there is a '|' to be considered but discarded
        strcpy (IO_msg, tmp_msg);
    }
    
}

int main (int argc, char ** argv) {


    signal (SIGUSR1, &watchdog_req);
    
    //in argv are the fds to communicate with server for obstacles and drone position and with map/kb process for the keyboard input
    
    if (argc < 3) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    

    int count = 0;
    const int maxcount = 2;
    char IO_msg [160];
    struct timespec kb_wait_time;

    //variables for the euler mehtod for the dynamics simulation
    //0 is position of previous iteration, 1 is position of 2 iterations ago
    double drone_x [2], drone_y [2];

    //position in current iteration of the simulation
    double v_x, v_y;
    struct position drone_act;
    double x_start = MAP_X_SIZE / 2;
    double y_start = MAP_Y_SIZE / 2;

    struct position obs_pos [obs_num];
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i].x = obs_pos [i].y = 0;
    }
    int obs_placed = 0;


    struct position targ_pos [targ_num];
    int targ_placed;
    int curr_targ = 0;

    //initialisation of drone position on the map (roughly at the center); 
    drone_act.x = x_start;
    drone_act.y = y_start;

    drone_x [0] = drone_x [1] = drone_act.x;
    drone_y [0] = drone_y [1] = drone_act.y;

    
    int syscall_res;
    int fd_out = 0, fd_wtch = 0, fd_param = 0; 

    

    fd_wtch = open (log_file [log_id], O_WRONLY, 0666);
    if (fd_wtch < 0) {
        perror ("log pipe open");
        exit (EXIT_FAILURE);
    }

    char logdata [10];
    int pid = getpid ();
    //printf ("%d\n", pid);

    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("pid formatting");
        exit (EXIT_FAILURE);
    }

    if (write (fd_wtch, logdata, sizeof (logdata)) < 0) {
        perror ("watchdog write");

        exit (EXIT_FAILURE);
    }

    char * param_file = "parameters.txt";

    //string in which read parameters of simulation;
    // 1 is force from keyboard, 2 is mass, 3 is friction coefficient, 
    //4 is obstacle / wall force modifier and 5 is max distance from obstacle to perceive force
    char param_str [8 * sizeof(double)]; 

    
    
    double ro_max;                      //maximum distance to perceive force from walls/obstacles
    double wall_dist_x, wall_dist_y;    //distance of drone from walls (used for right wall and low wall)

    double M, K, Eta, Epsilon; 

    double ro_targ_1, ro_targ_2;

    if ((fd_param = open (param_file, O_RDONLY)) < 0) {
        perror ("param open");
        exit (EXIT_FAILURE);
    }
    
    int fd_serv_in, fd_serv_out, fd_in_kb;
    sscanf (argv [1], "%d %d", &fd_serv_in, &fd_serv_out);
    sscanf (argv [2], "%d", &fd_in_kb);
    

    int reset, quit, obs_targ_res_count = 0;

    double Force_x, Force_y, kb_force_x, kb_force_y, v_ipot;

    double obs_force_x, obs_force_y, obs_force, targ_force_x, targ_force_y, targ_force; 

    double wall_force_x, wall_force_y;

    double dist_obs_x, dist_obs_y, dist_obs;

    double dist_targ_x, dist_targ_y, dist_targ;

    struct timespec delta_time, rem_time;

    long int nsec_diff;

    long int time_to_sleep;

    const int wxsize = MAP_X_SIZE, wysize = MAP_Y_SIZE;

    //reading from the (build/)parameters.txt file the parameters to be used in the simulation
    syscall_res = lseek (fd_param, 0, SEEK_SET);
    if (syscall_res < 0) {
        perror ("lseek");
        exit (EXIT_FAILURE);
    }
    syscall_res = read (fd_param, param_str, 8 * sizeof (double));
    if (syscall_res < 0) {
        perror ("param_read 1:");
        exit (EXIT_FAILURE);
    }

    syscall_res = sscanf (param_str, "%lf %lf %lf %lf %lf %lf %lf %lf", &PROPULSION_STEP, &M, &K, &Eta, &ro_max, &Epsilon, &ro_targ_1, &ro_targ_2);
    if (syscall_res < 0) {
        perror ("param_scan 1:");
        exit (EXIT_FAILURE);
    }
    double Max_Force= PROPULSION_STEP;
    Max_Force *= maxforceratio;

    //multiplying by frequency instead of dividing by dt to avoid getting nan as result
    //values used in differential equation of drone
    double K_1 = M * frequency * frequency;
    double K_2 = K * frequency;
    int kb_in;


    //blocking read waiting for the server to be ready
    printf ("waiting for message from server\n");
    if (read (fd_serv_in, IO_msg, 160) < 0) {
        perror ("wait read");
        exit (EXIT_FAILURE);
    }
    printf ("message received\n starting operations\n");
    printf ("sending initials positions to server\n");
    update_BB (drone_act, fd_serv_out, fd_serv_in, IO_msg);

    sigset_t select_mask, orig_mask;
    sigaddset (&select_mask, SIGUSR1);
    fd_set kb_reception;

    while (1) {
            /*input informations*/

        //keyboard input

        //setting up a pselect because of different frequencies of the dynamics and the map processes 
        kb_wait_time.tv_sec = 0;
        kb_wait_time.tv_nsec = 0;
        FD_ZERO (&kb_reception);
        FD_SET (fd_in_kb, &kb_reception);
        syscall_res = pselect (fd_in_kb + 1, &kb_reception, NULL, NULL, &kb_wait_time, &select_mask);
        if (syscall_res < 0) {
            perror ("kb pselect");
            exit (EXIT_FAILURE);
        }
        kb_in = 0;
        //printf ("reading from map\n");
        if (syscall_res > 0) {
            if (read (fd_in_kb, IO_msg, 160) < 0) {
                perror ("kb read");
                exit (EXIT_FAILURE);
            }
            if (sscanf (IO_msg, "%d", &kb_in) < 0) {
                perror ("kb sscanf");
                exit (EXIT_FAILURE);
            }
        }
        //printf ("read keyboard press\n");
        
        //server (obstacles and targets) input, read only every 10 cycles
        if (obs_targ_res_count % 10 == 0) {
            request_obstacles (fd_serv_in, fd_serv_out, IO_msg, &obs_placed, obs_pos);
            request_targets (fd_serv_in, fd_serv_out, IO_msg, &targ_placed, targ_pos);
            obs_targ_res_count = 0;
        }


                                /*forces acting on drone*/

        //forces for keyboard input (received from the process displaying the map)
        Get_Kb_In (kb_in, &quit, &reset, &kb_force_x, &kb_force_y);

        //commands related to keyboard input although not related to  forces

        if (reset == 1) {//reset drone to starting position and wait for new obstacles generated
            drone_act.x = drone_x [0] = drone_x [1] = x_start;
            drone_act.y = drone_y [0] = drone_y [1] = y_start;
            int time_left = sleep (1);
            while (time_left > 0) time_left = sleep (time_left);
            request_obstacles (fd_serv_in, fd_serv_out, IO_msg, &obs_placed, obs_pos);
            request_targets (fd_serv_in, fd_serv_out, IO_msg, &targ_placed, targ_pos);
            obs_targ_res_count = 0; 
            curr_targ = 0;           
        }

        if (quit == 1) {
            exit (EXIT_SUCCESS);
        }

        //obstacles
        obs_force_x = 0;
        obs_force_y = 0;
        for (int i = 0; i < obs_placed; i++) {
            dist_obs_x = drone_act.x - obs_pos [i].x;
            dist_obs_y = drone_act.y - obs_pos [i].y;
            
            dist_obs = sqrt (dist_obs_x * dist_obs_x + dist_obs_y * dist_obs_y);
            if (dist_obs < ro_max) {
                obs_force = Eta * (1/dist_obs - 1/ro_max) / (dist_obs * dist_obs); 
                if (obs_force > Max_Force) obs_force = Max_Force;
                if (obs_force < -Max_Force) obs_force = - Max_Force;
                /* dist_obs_x / dist_obs is the x component of the gradient of the distance vector, same logic for y component*/
                obs_force_x += obs_force * dist_obs_x / dist_obs;
                obs_force_y += obs_force * dist_obs_y / dist_obs;
            }
        }
        

        //targets
        targ_force_x = 0;
        targ_force_y = 0;
        //printf ("%d\n", curr_targ);
        
        for (int i = curr_targ; i < targ_placed; i++) {
            dist_targ_x = drone_act.x - targ_pos [i].x;
            dist_targ_y = drone_act.y - targ_pos [i].y;
            
            //errno = 0;
            //feclearexcept (FE_ALL_EXCEPT);
            dist_targ = sqrt (dist_targ_x * dist_targ_x + dist_targ_y * dist_targ_y);
            /*if (errno != 0) {
                perror ("sqrt targ");
                exit (EXIT_FAILURE);
            }/**/
            /*if (i == curr_targ) {
                printf ("%d %lf\n", curr_targ, dist_targ);
                //printf ("%lf %lf\n%lf %lf\n", drone_act.x, drone_act.y, targ_pos [i].x, targ_pos [i].y);
            }/**/
            //detection of current target touched
            if (i == curr_targ & dist_targ <= TARG_TOUCH) {
                curr_targ ++;
                printf ("reached current target! well done!\a\n");
                /*update on blackboard of current target to reach*/
                /*if (sprintf (IO_msg, "%c%d", 'u', curr_targ) < 0) {
                    perror ("taret reach message");
                    exit (EXIT_FAILURE);
                }
                if (write (fd_serv_out, IO_msg, strlen (IO_msg) + 1) < 0) {
                    perror ("target reach write");
                    exit (EXIT_FAILURE);
                }/**/
                if (dprintf (fd_serv_out, "%c %d\n", 'u', curr_targ) < 0) {
                    perror ("target update sending");
                }
                if (read (fd_serv_in, IO_msg, 160) < 0) {
                    perror ("target reached read");
                }
                continue;
            }

            if (dist_targ < ro_targ_1) {
                //targ_force = Eta * (1/dist_targ - 1/ro_max) / (dist_targ * dist_targ); 
                
                /* dist_targ_x / dist_targ is the x component of the gradient of the distance vector, same logic for y component*/
                //targ_force_x += targ_force * dist_targ_x / dist_targ;
                //targ_force_y += targ_force * dist_targ_y / dist_targ;
                targ_force_x += - Epsilon * dist_targ_x;
                targ_force_y += - Epsilon * dist_targ_y;
            } else if (dist_targ < ro_targ_2) {
                targ_force_x += - Epsilon * dist_targ_x / dist_targ;
                targ_force_y += - Epsilon * dist_targ_y / dist_targ;
            }
        }

        //walls 
        wall_force_x = 0;
        wall_force_y = 0;
        //values for right and top walls, for left and low walls (0 x and y respectively as coordinates) used directly drone_act
        wall_dist_x = MAP_X_SIZE - drone_act.x;
        wall_dist_y = MAP_Y_SIZE - drone_act.y;
        if (drone_act.x < ro_max) { //left end, force to the right
            wall_force_x = Eta * (1/drone_act.x - 1/ro_max) /(drone_act.x * drone_act.x);
        }
        if (drone_act.y < ro_max) { //lower end, force upward
            wall_force_y = Eta * (1/drone_act.y - 1/ro_max) /(drone_act.y * drone_act.y);
        }
        if (wall_dist_x < ro_max) {//right edge, force towards the right
            wall_force_x = - Eta * (1/wall_dist_x - 1/ro_max) /(wall_dist_x * wall_dist_x);
        }
        if (wall_dist_y < ro_max) {//upper edge, force downward
            wall_force_y = - Eta * (1/wall_dist_y - 1/ro_max) /(wall_dist_y * wall_dist_y);
        }

        //total
        Force_x = kb_force_x + obs_force_x + targ_force_x + wall_force_x;
        Force_y = kb_force_y + obs_force_y + targ_force_y + wall_force_y;


                                    /*physics step simulation*/
        //euler method: F = M* (xi-2 + xi - 2*xi-1)/dt^2 + K(xi - xi-1)/dt     
        //using euler formula for current x position

        drone_act.x = (Force_x + (2 * K_1 + K_2) * drone_x [0] - K_1 * drone_x [1]) / (K_1 + K_2);

        //update x positions of previous iterations

        drone_x [1] = drone_x [0];
        drone_x [0] = drone_act.x;
        
        //euler formula for current y position

        drone_act.y = (Force_y + (2 * K_1 + K_2) * drone_y [0] - K_1 * drone_y [1]) / (K_1 + K_2);

        //update positions of previous iterations

        drone_y [1] = drone_y [0];
        drone_y [0] = drone_act.y;

        //send to blackboard / map displayer the just computed location of the drone

        if (count == 0) {
            update_BB (drone_act, fd_serv_out, fd_serv_in, IO_msg);
        }

        count = (count + 1) % maxcount;
        
        //wait for beginning of next period
        usleep (SEC_TO_USEC / frequency);
    }

    exit (EXIT_FAILURE); //failure because program should never end here
    return 0;
}