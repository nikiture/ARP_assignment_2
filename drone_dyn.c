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

double PROPULSION_STEP = 1;
const double root2 = 1.41421;
const int frequency = 100;               //frequency of dynamycs computation in hertz


const int log_id = 0; //position in log_file where pid is written

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

void Terminate_all (int fd_1, int fd_2, int fd_3, int fd_4, void * pos_writer, sem_t * map_semaph, sem_t * kb_sem, void * kb_shm, int exit_status) {
    if (fd_1 > 0) close (fd_1);
    if (fd_2 > 0) close (fd_2);
    if (fd_3 > 0) close (fd_3);
    if (fd_4 > 0) close (fd_4);

    munmap (pos_writer, 2 * sizeof (double));

    munmap (kb_shm, sizeof (int));

    sem_close (map_semaph);

    sem_close (kb_sem);

    sem_unlink (KB_SEM);

    sem_unlink (MAP_SEM);

    exit (exit_status);
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
int update_BB (double * pos, int out_fd, int in_fd, char * IO_msg) {
/*int update_BB (double * pos, sem_t * map_semaph, void * pos_writer) {*/

    /*if ((sem_wait (map_semaph)) < 0) {
        perror ("semaph taking");
        Terminate_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }

    void * syscall_res = memcpy (pos_writer, pos, 2 * sizeof (double));
    if (syscall_res == NULL) {
        perror ("shared memory writing");
        Terminate_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }
    if (sem_post (map_semaph) < 0) {
        perror ("semaph release");
        Terminate_all (-1, -1, -1, -1, pos_writer, map_semaph, NULL, NULL, EXIT_FAILURE);
    }*/
    //printf ("dynamics: preparing message\n");
    sprintf (IO_msg, "%lf3 %lf3", pos [0], pos [1]);
    //printf ("dynamic: sending message\n");
    write (out_fd, IO_msg, strlen (IO_msg) + 1);
    //printf ("dynamic: waiting for response\n");
    read (in_fd, IO_msg, 160);
    //printf ("dynamics: received response from server\n");
}
void request_obstacles (int fd_in, int fd_out, char * IO_msg, int * obs_placed, double obs_pos [obs_num] [2]) {
    char tmp_msg [160];
    //printf ("dynamics: creating request message\n");
    if (sprintf (IO_msg, "%c", 'r') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    //printf ("dynamics: sending request message\n");
    if (write (fd_out, IO_msg, strlen (IO_msg)) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    //printf ("dynamics: waiting for response from server\n");
    if (read (fd_in, IO_msg, 160) < 0) {
        perror ("response read");
        exit (EXIT_FAILURE);
    }
    //printf ("dynamics: processing response\n");
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
    /*for (int i = 0; i < *obs_placed; i ++) {
        printf ("%lf %lf\n", obs_pos [i] [0], obs_pos [i] [1]);
    }*/
}

int main (int argc, char ** argv) {
    //printf ("dynaics: %d\n", getpid());
    //in argv are the fds to communicate with server for obstacles and drone position
    signal (SIGUSR1, watchdog_req);
    if (argc < 3) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    /*sem_t * map_semaph = sem_open (MAP_SEM, O_CREAT, 0666, 1);
    if (map_semaph == SEM_FAILED) {
        perror ("semaphore opening");
        exit (EXIT_FAILURE);
    }

    if (sem_init (map_semaph, 0, 1) < 0) {
        perror ("semaphore initialisation");
        sem_close (map_semaph); 
        exit (EXIT_FAILURE);
    }*/

    sem_t * kb_sem = sem_open (KB_SEM, O_CREAT, 0666, 1);
    if (kb_sem == SEM_FAILED) {
        perror ("semaphore creation kb");
        //sem_close (map_semaph);
        exit (EXIT_FAILURE);
    }

    if (sem_init (kb_sem, 0, 1) < 0) {
        perror ("semaphore kb creation");
        //sem_close (map_semaph);
        sem_close (kb_sem);
        exit (EXIT_FAILURE);
    }

    int count = 0;
    const int maxcount = 2;
    char IO_msg [160];
    struct timespec start_time;
    struct timespec end_time;

    //0 is position of previous iteration, 1 is position of 2 iterations ago
    double drone_x [2], drone_y [2];

    //position in current iteration of the simulation
    double drone_act [2], v_x, v_y;
    double x_start = MAP_X_SIZE / 2;
    double y_start = MAP_Y_SIZE / 2;

    double obs_pos [obs_num] [2];
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i] [0] = obs_pos [i] [1] = 0;
    }
    int obs_placed = 0;

    //initialisation of drone position on the map (roughly at the center); 
    drone_act [0] = x_start;
    drone_act [1] = y_start;

    drone_x [0] = drone_x [1] = drone_act [0];
    drone_y [0] = drone_y [1] = drone_act [1];

    
    int syscall_res;
    int fd_out = 0, fd_wtch = 0, fd_param = 0; 

    

    fd_wtch = open (log_file [log_id], O_WRONLY, 0666);
    if (fd_wtch < 0) {
        perror ("log pipe open");
        Terminate_all (-1, -1, -1, fd_wtch, NULL, NULL, kb_sem, NULL, EXIT_FAILURE); //null because shared memories not yet initialised
    }

    char logdata [10];
    int pid = getpid ();
    //printf ("%d\n", pid);

    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("pid formatting");
        Terminate_all (-1, fd_wtch, -1, -1, NULL, NULL, kb_sem, NULL, EXIT_FAILURE);
    }

    if (write (fd_wtch, logdata, sizeof (logdata)) < 0) {
        perror ("watchdog write");

        Terminate_all (-1, fd_wtch, -1, -1, NULL, NULL, kb_sem, NULL, EXIT_FAILURE);
    }

    char * param_file = "parameters.txt";

    //string in which read parameters of simulation;
    // 1 is force from keyboard, 2 is mass, 3 is friction coefficient, 
    //4 is obstacle / wall force modifier and 5 is max distance from obstacle to perceive force
    char param_str [5 * sizeof(double)]; 

    
    
    double ro_max;                      //maximum distance to perceive force from walls/obstacles
    double wall_dist_x, wall_dist_y;    //distance of drone from walls (used for right wall and low wall)

    double M, K, Eta; 

    if ((fd_param = open (param_file, O_RDONLY)) < 0) {
        perror ("param open");
        Terminate_all (-1, -1, fd_wtch, -1, NULL, NULL, kb_sem, NULL, EXIT_FAILURE);
    }

    int kb_fd = shm_open (KB_ADDR, O_RDWR, 0666);
    if (kb_fd < 0) {
        perror ("shared memory opening kb");
        Terminate_all (-1, fd_param, fd_wtch, -1, NULL, NULL, kb_sem, NULL, EXIT_FAILURE);
    }

    void * kb_ptr = mmap (0, sizeof (int), PROT_WRITE | PROT_READ, MAP_SHARED, kb_fd, 0);
    if (kb_ptr == MAP_FAILED) {
        perror ("shared memory kb mapping");
        Terminate_all (-1, fd_wtch, fd_param, kb_fd, NULL, NULL, kb_sem, NULL, EXIT_FAILURE);
    }
    
    fd_out = shm_open (MAP_ADDR, O_CREAT | O_RDWR, 0666);
    if (fd_out < 0) { 
        perror ("open");
        Terminate_all (fd_out, fd_wtch, fd_param, kb_fd, NULL, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }

    char pos_str [2 * sizeof (double)];
    void * pos_writer = mmap (0, 2 * sizeof (double), PROT_WRITE | PROT_READ, MAP_SHARED, fd_out, 0);
    if (pos_writer == MAP_FAILED) {
        perror ("shm mapping");
        Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, NULL, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    
    int fd_serv_in, fd_serv_out, fd_in_kb;
    sscanf (argv [1], "%d %d", &fd_serv_in, &fd_serv_out);
    sscanf (argv [2], "%d", &fd_in_kb);
    
    //update_BB (drone_act, map_semaph, pos_writer);

    int reset, quit, obs_res_count = 0;

    double Force_x, Force_y, kb_force_x, kb_force_y, v_ipot;

    double obs_force_x, obs_force_y, obs_force, targ_force_x, targ_force_y; 

    double wall_force_x, wall_force_y;

    double dist_obs_x, dist_obs_y, dist_obs;

    struct timespec delta_time, rem_time;

    long int nsec_diff;

    long int time_to_sleep;

    const int wxsize = MAP_X_SIZE, wysize = MAP_Y_SIZE;

    syscall_res = lseek (fd_param, 0, SEEK_SET);
    if (syscall_res < 0) {
        perror ("lseek");
        Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    syscall_res = read (fd_param, param_str, 5 * sizeof (double));
    if (syscall_res < 0) {
        perror ("param_read 1:");
        Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }

    syscall_res = sscanf (param_str, "%lf %lf %lf %lf %lf", &PROPULSION_STEP, &M, &K, &Eta, &ro_max);
    if (syscall_res < 0) {
        perror ("param_scan 1:");
        Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    //multiplying by frequency instead of dividing by dt to avoid getting nan as result
    //values used in differential equation of drone
    double K_1 = M * frequency * frequency;
    double K_2 = K * frequency;
    int kb_in;
    void * memcopy_res;

    //blocking read waiting for the server to be ready
    printf ("waiting for message from server\n");
    if (read (fd_serv_in, IO_msg, 160) < 0) {
        perror ("wait read");
        Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
    }
    

    while (1) {
            /*input informations*/

        //keyboard input
        /*
        if (sem_wait (kb_sem) < 0) {
            perror ("kb semaphore taking");
            Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
        }

        memcopy_res = memcpy (&kb_in, kb_ptr, sizeof (int));
        if (memcopy_res == NULL) {
            perror ("shared memory kb write");
            Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
        }

        if (sem_post (kb_sem) < 0) {
            perror ("kb semaphore release");
            Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE);
        }
        */

        if (read (fd_in_kb, IO_msg, 160) < 0) {
            perror ("kb read");
            exit (EXIT_FAILURE);
        }
        if (sscanf (IO_msg, "%d", &kb_in) < 0) {
            perror ("kb sscanf");
            exit (EXIT_FAILURE);
        }
        //printf ("dynamics: received from keyboard %d\n", kb_in);
        
        //server (obstacles) input
        if (obs_res_count % 100 == 0) {
            request_obstacles (fd_serv_in, fd_serv_out, IO_msg, &obs_placed, obs_pos);
            //printf ("read obstacle positions\n");
            obs_res_count = 0;
        }

                                /*forces acting on drone*/
        //forces for keyboard input (received from the process displaying the map)
        Get_Kb_In (kb_in, &quit, &reset, &kb_force_x, &kb_force_y);

        //commands related to keyboard input although not related to  forces

        if (reset == 1) {//reset drone to starting position
            drone_act [0] = drone_x [0] = drone_x [1] = x_start;
            drone_act [1] = drone_y [0] = drone_y [1] = y_start;
            int time_left = sleep (1);
            while (time_left > 0) time_left = sleep (time_left);
            request_obstacles (fd_serv_in, fd_serv_out, IO_msg, &obs_placed, obs_pos);
            //printf ("read obstacle positions\n");
            obs_res_count = 0;            
        }

        if (quit == 1) {
            Terminate_all (fd_out, fd_param, fd_wtch, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_SUCCESS);
        }

        //obstacles
        obs_force_x = 0;
        obs_force_y = 0;
        for (int i = 0; i < obs_placed; i++) {
            dist_obs_x = drone_act [0] - obs_pos [i] [0];
            dist_obs_y = drone_act [1] - obs_pos [i] [1];
            
            dist_obs = sqrt (dist_obs_x * dist_obs_x + dist_obs_y * dist_obs_y);
            //printf ("%lf %lf %lf %lf %lf %lf\n", drone_act [0], drone_act [1], obs_pos [i] [0], obs_pos [i] [1], dist_obs_x, dist_obs_y);
            //printf (\n", drone_act [1], obs_pos [i] [1], dist_obs_y);
            //printf ("%lf\n", dist_obs);
            if (dist_obs < ro_max) {
                //printf ("%lf %lf\n", dist_obs_x, dist_obs_y);
                obs_force = Eta * (1/dist_obs - 1/ro_max) / (dist_obs * dist_obs); 
                /* dist_obs_x / dist_obs is the x component of the gradient of the distance vector*/
                obs_force_x += obs_force * dist_obs_x / dist_obs;
                obs_force_y += obs_force * dist_obs_y / dist_obs;
            }
        }
        

        //targets
        targ_force_x = 0;
        targ_force_y = 0;

        //walls 
        wall_force_x = 0;
        wall_force_y = 0;

        wall_dist_x = MAP_X_SIZE - drone_act [0];
        wall_dist_y = MAP_Y_SIZE - drone_act [1];
        if (drone_act [0] < ro_max) { //left end, force to the right
            wall_force_x = Eta * (1/drone_act [0] - 1/ro_max) /(drone_act [0] * drone_act [0]);
        }
        if (drone_act [1] < ro_max) { //lower end, force upward
            wall_force_y = Eta * (1/drone_act [1] - 1/ro_max) /(drone_act [1] * drone_act [1]);
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

        drone_act [0] = (Force_x + (2 * K_1 + K_2) * drone_x [0] - K_1 * drone_x [1]) / (K_1 + K_2);

        //update x positions of previous iterations

        drone_x [1] = drone_x [0];
        drone_x [0] = drone_act [0];
        
        //euler formula for current y position

        drone_act [1] = (Force_y + (2 * K_1 + K_2) * drone_y [0] - K_1 * drone_y [1]) / (K_1 + K_2);

        //update positions of previous iterations

        drone_y [1] = drone_y [0];
        drone_y [0] = drone_act [1];

        //send to blackboard / map displayer the just computed location of the drone
        //update_BB (drone_act, map_semaph, pos_writer);
        if (count == 0) {
            update_BB (drone_act, fd_serv_out, fd_serv_in, IO_msg);
            //printf ("dynamics: sent position to server\n");
        }

        count = (count + 1) % maxcount;
        
        //wait for beginning of next period
        usleep (SEC_TO_USEC / frequency);
    }

    Terminate_all (fd_out, fd_wtch, fd_param, kb_fd, pos_writer, NULL, kb_sem, kb_ptr, EXIT_FAILURE); //failure because program should never end here
    return 0;
}