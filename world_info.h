#define MAP_ADDR "/map"
#define MAP_SEM "/sem_map"
#define KB_ADDR "/kb_mem"
#define KB_SEM "/sem_kb"
//data size: 2 double, 2 double per obstacle
#define DATA_DIM 2 * sizeof (double);
#define SHM_FORMAT "%lf %lf"
#include <ncurses.h>
#define MAP_X_SIZE 130.0
#define MAP_Y_SIZE 40.0
#define LOG_SEM "/log_sem"
#define TARG_TOUCH 1

const char * log_file [5] = {"/tmp/process_status_0", "/tmp/process_status_1", "/tmp/process_status_2", "/tmp/process_status_3", "/tmp/process_status_4"};

#define obs_num 10
#define targ_num 5
#define SEC_TO_USEC 1000000

struct position {
    double x;
    double y;
};

