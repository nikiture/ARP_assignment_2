#define MAP_ADDR "/map"
#define MAP_SEM "/sem_map"
#define KB_ADDR "/kb_mem"
#define KB_SEM "/sem_kb"
//data size: 2 double, 2 double per obstacle
#define DATA_DIM 2 * sizeof (double);
#define SHM_FORMAT "%lf %lf"
#include <ncurses.h>
#define MAP_X_SIZE 120
#define MAP_Y_SIZE 30
#define LOG_SEM "/log_sem"

const char * log_file [5] = {"/tmp/process_status_0", "/tmp/process_status_1", "/tmp/process_status_2", "/tmp/process_status_3", "/tmp/process_status_4"};

const int obs_num = 10;
const int targ_num = 5;