/* Author: Austin Bathgate
 *  Last Date Modified: Mar 1, 2026
 *
 *  Brief: This script is made to complete the requirements of exercise 3 part 2. It is designed to show the use of a mutex lock
 *  between one thread updating data and a second, higher priotity thread reading it. Two threads will be implemented in total in this script, one
 *  as an update thread at 1 Hz, and a second will be a read thread reading at 0.1 Hz. From RM, the update thread will have the highest priority.
 *  If the mutex is implemented proerly, the read thread will always agree with the update thread and we will never see mixed states. The data
 *  itself will be contained in a structure, and a timespec structure will be inside the larger structure. The data itself will be
 *  lat, long, alt, roll, pitch, yaw, timespec. Only one structure will be used, shared between the 2 threads, but a local copy will be made in the read thread
 *  before printing.
 *
 *  To display true real time behavior, threads will be locked to core 3. SCHED_FIFO will be used with main 99, read 98, update 97 priorities.
 *
 *  printf() is currently used since the read rate is rather slow at T = 10s and a printf() can easily fit in here even with the 1Hz interference
 *  from the update thread.
 *
 *  DATA STRUCTURE:  LAT     :   0.01X
 *                   LONG    :   0.2X
 *                   ALT     :   0.25X
 *                   ROLL    :   SIN(0.05*X)
 *                   PITCH   :   COS(0.05*X^2)
 *                   YAW     :   COS(0.05*X)
 *                   TIMESPEC:   time_t tv_sec - whole seconds
 *                               long tv_nsec  - nanoseconds
 *
 *  CODE REUSE AND REFERENCES: Code from exercise 1 was referenced heavily to aid in setting up threads pinned to cpu 3
 *  running as SCHED_FIFO. Particularly, my final code for exercise 1 and pthread_affinity.
 */

// LIBRARIES / INCLUDES
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L //allows change of mutex clock to CLOCK_MONOTONIC

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <errno.h>     //for system error calls
#include <pthread.h>   //for use of pthreads
#include <sched.h>     //sched policies
#include <sys/types.h> //OS types
#include <unistd.h>    //for getpid()

// DEFINE STATEMENTS
#define SCHED_POLICY SCHED_FIFO
#define READ_T_SEC (10)  // read thread period in seconds
#define UPDATE_T_SEC (1) // update thread period in seconds
#define END_TIME_S (180) // end time for program in seconds, relative to start
#define MAX_READS (18)   // max reads to store for dump at end of program, 18 since we get 18 reads at 0.1Hz in 180s

// POSIX ITEM DECLARATION
pthread_t main_thread;   // used with main thread to set as fifo pri 99, cpu 3
pthread_t update_thread; // 1Hz update thread (T = 1s), fifo pri 98, cpu 3
pthread_t read_thread;   // 0.1Hz read (T = 10s), fifo 97, cpu 3

pthread_mutex_t data_mutex;          // mutex to protect shared data structure
pthread_mutexattr_t data_mutex_attr; // attribute structure for mutex

pthread_attr_t fifo_sched_attr; // attribute structure to use with threads
struct sched_param fifo_param;  // structure which simply contains priority

// TIME ITEM DECLARATION
struct timespec time_start; // global timespec to log start time of program for relative time calculations

// MAIN DATA STRUCTURE
typedef struct
{
    double lat;                // latitude
    double lon;                // longitude
    double alt;                // altitude
    double roll;               // roll
    double pitch;              // pitch
    double yaw;                // yaw
    struct timespec timestamp; // timestamp of last update to this data structure
    int sequence_count;        // sequence count of update or thread for verification
} sensor_data_t;

sensor_data_t shared_data; // global instance of data structure to be shared between threads
sensor_data_t read_summary[18]; // reads stored for dump at end of program


//*************SCHEDULER FUNCTIONS****************
// CODE REUSE: The print_scheduler function of exercise 1 was reused to verify schedule.
/*all treads in a process share sched policy, by using 3 lines below,
we pull policy for one which is applicable to all*/
void print_scheduler(void)
{
    int schedType = sched_getscheduler(getpid());
    // switch int values linked to macros from <sched.h>
    switch (schedType)
    {
    case SCHED_FIFO:
        printf("Pthread policy is SCHED_FIFO\n");
        break;
    case SCHED_OTHER:
        printf("Pthread policy is SCHED_OTHER\n");
        break;
    case SCHED_RR:
        printf("Pthread policy is SCHED_RR\n");
        break;
    default:
        printf("Pthread policy is UNKNOWN\n");
    }
}

/* CODE REUSE: This function sets the caller (main) to be fifo pri 99, and it
 *  configures the fifo_sched_attr attribute structure to have the cpu pin 3, fifo 99 priority
 */
void set_scheduler(void)
{
    int max_prio, scope, rc, cpuidx;
    cpu_set_t cpuset;

    printf("INITIAL ");
    print_scheduler();

    pthread_attr_init(&fifo_sched_attr);                                    // init attribute object of type pthread_attr_t for our fifo sched params
    pthread_attr_setinheritsched(&fifo_sched_attr, PTHREAD_EXPLICIT_SCHED); // explicitly use selected sched and params below
    pthread_attr_setschedpolicy(&fifo_sched_attr, SCHED_POLICY);            // selected sched = SCHED_FIFO (line 32 define)
    // cpu pin to cpu 3
    CPU_ZERO(&cpuset);                                                         // zero out CPU SET to remove default artifacts
    cpuidx = (3);                                                              // CPU3
    CPU_SET(cpuidx, &cpuset);                                                  // add CPU3 to usable CPUs (CPU set)
    pthread_attr_setaffinity_np(&fifo_sched_attr, sizeof(cpu_set_t), &cpuset); // ties cpu3 pin to our fifo_sched_attr

    // assign max priority to fifo_param structure
    max_prio = sched_get_priority_max(SCHED_POLICY); // max prio of SCHED_FIFO is 99
    fifo_param.sched_priority = max_prio;

    // process level - sets process scheduling polifcy to
    if ((rc = sched_setscheduler(getpid(), SCHED_POLICY, &fifo_param)) < 0)
        perror("sched_setscheduler");
    // attribute level - when fifo_sched_attr is used in p-thread create, it will have this cpu3 pin, fifo, max prio attributes
    pthread_attr_setschedparam(&fifo_sched_attr, &fifo_param);

    printf("ADJUSTED ");
    print_scheduler();
}

//*************TIME FUNCTIONS****************
/* Handling time was one of my large questions, will log a start time then compute relative time. Full time stamps will be printed,
 *  but the x value fed to the functions used for data generation will be the relative time in seconds only for easy verification.
 *  Turns out this is simlpe enough that I do not need helpers.
 */

/* Helper function to compute the difference between two timespecs, used for relative time calculations and printing.
 *
 *  INPUTS:  start - timespec representing the start time
 *           end   - timespec representing the end time
 *  OUTPUTS: temp  - timespec representing the difference between end and start (end - start)
 */
struct timespec timespec_diff(const struct timespec *start, const struct timespec *end)
{
    struct timespec temp;
    /* if borrow, subtract additional 1 from seconds, and add 1 second worth of nanoseconds to the nanosec field,
       then subtract as normal for nanosecopnds*/
    if ((end->tv_nsec - start->tv_nsec) < 0)
    {
        temp.tv_sec = end->tv_sec - start->tv_sec - 1;
        temp.tv_nsec = 1000000000 + end->tv_nsec - start->tv_nsec;
    }
    // else standard subtraction for both fields
    else
    {
        temp.tv_sec = end->tv_sec - start->tv_sec;
        temp.tv_nsec = end->tv_nsec - start->tv_nsec;
    }
    return temp;
}

//*************THREAD FUNCTIONS****************
/* The update thread will run at 1Hz, and will update the shared data structure with new values based on the equations in the brief. The read thread will run at 0.1Hz, and will read the shared data structure and print it out.
 *  Both threads will use the same mutex to protect the shared data structure. From RM theory, the update thread will have the hoighest priority.
 */
void *updateThread(void *threadp)
{
    // local vars
    struct timespec time_relative; // time since start for timestamping and data generation
    struct timespec next_release;
    struct timespec time_now; // current time for relative time calculations
    int time_s = 0;           // integer seconds from the timespec for use in data generation
    int i = 0;                // loop counter

    // initialize next_release to now for first run
    clock_gettime(CLOCK_MONOTONIC, &next_release); // get current time for absolute sleep calculations
    next_release.tv_sec += UPDATE_T_SEC;              // set first release to be 1s after start

    while (1)
    {
        // PERIODIC GATE
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_release, NULL); // sleep until next release time
        i++;                                                                  // increment sequence count
        next_release.tv_sec += UPDATE_T_SEC;                                  // calculate next release, need only s field since we are at 1Hz

        // print iteration and timestamp, calculate relative time
        clock_gettime(CLOCK_MONOTONIC, &time_now);             // get current time for relative time calculations and timestamping
        time_relative = timespec_diff(&time_start, &time_now); // compute relative time since start for timestamping and data generation
        time_s = (int)(time_relative.tv_sec);                  // integer seconds since start
        printf("->-> Begin update %d at relative time: %ld.%09ld sec, absolute time: %ld.%09ld sec\n", (i), time_relative.tv_sec, time_relative.tv_nsec, time_now.tv_sec, time_now.tv_nsec);
        printf("using integer time in data generation: %d seconds\n\n", time_s);
        // lock mutex to update shared data structure
        pthread_mutex_lock(&data_mutex);

        // update shared data structure with new values based on equations in brief
        shared_data.lat = 0.01 * time_s;
        shared_data.lon = 0.2 * time_s;
        shared_data.alt = 0.25 * time_s;
        shared_data.roll = sin(0.05 * time_s);
        shared_data.pitch = cos(0.05 * pow(time_s, 2));
        shared_data.yaw = cos(0.05 * time_s);
        shared_data.timestamp = time_relative;
        shared_data.sequence_count = i;

        pthread_mutex_unlock(&data_mutex); // unlock mutex after update

        if (time_relative.tv_sec >= END_TIME_S) // end after 181 seconds to ensure we get the final read at 180s
        {
            printf("Update thread reached or exceeded end time of %d seconds, exiting...\n", END_TIME_S);
            break;
        }
    }
}

void *readThread(void *threadp)
{

    struct timespec time_relative; // time since start for timestamping and data generation
    struct timespec time_now;      // current time for relative time calculations
    struct timespec next_release;  // next release time for periodic gate
    int i = 0;                     // sequence counter
    sensor_data_t local_copy;     // local copy of shared data structure for printing
    clock_gettime(CLOCK_MONOTONIC, &next_release); // get current time for absolute sleep calculations
    next_release.tv_sec += READ_T_SEC; // set first release to be 10s after start
    while (1)
    {
        // PERIODIC GATE
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_release, NULL); // sleep until next release time                                                              // increment sequence count
        i++; // preincrement so all begins at 1
        next_release.tv_sec += READ_T_SEC;                                    // calculate next release, need only s field since we are at 0.1Hz

        //RELATIVE TIME CALC AND PRINT
        // print iteration and timestamp, calculate relative time
        clock_gettime(CLOCK_MONOTONIC, &time_now);             // get current time for relative time calculations and timestamping
        time_relative = timespec_diff(&time_start, &time_now); // compute relative time since start for timestamping and data generation
        printf("<-<- Begin read %d at relative time: %ld.%09ld sec, absolute time: %ld.%09ld sec\n", (i), time_relative.tv_sec, time_relative.tv_nsec, time_now.tv_sec, time_now.tv_nsec);

        //DATA COPY CRITICAL SECTION
        // lock mutex to copy shared data structure
        pthread_mutex_lock(&data_mutex);
        // make local copy of shared data structure for printing
        local_copy = shared_data;
        // unlock mutex after read
        pthread_mutex_unlock(&data_mutex); 

        printf("***Read values from int seconds of relative time\n{\n lat=%f\n lon=%f\n alt=%f\n roll=%f\n pitch=%f\n yaw=%f\n write_timestamp=%ld.%09ld\n seq_count=%d\n}\n\n",
               local_copy.lat,
               local_copy.lon,
               local_copy.alt,
               local_copy.roll,
               local_copy.pitch,
               local_copy.yaw,
               local_copy.timestamp.tv_sec,
               local_copy.timestamp.tv_nsec,
               local_copy.sequence_count);
        
        // store read for dump at end of program, only store if we have storage capacity to avoid overflow
        // no shared access until after all joins so no mutex needed
        if(i < MAX_READS+1) //+1 due to preincrement scheme
        {
            read_summary[i-1] = local_copy;
        }   // store read for dump at end of program
        else{
            printf("Read count exceeded storage capacity, not storing this read for dump at end of program.\n");
        }

        if (time_relative.tv_sec >= END_TIME_S) // end after 180 seconds to ensure we get the final read at 180s
        {
            printf("Read thread reached end time of %d seconds, exiting...\n", END_TIME_S);
            break;
        }
    }
}

int main(int argc, char *argv[])
{
    int rc;
    int i, j;
    cpu_set_t cpuset;

    //*********WELCOME MESSAGE *******/
    printf("*****Starting Exercise 3 Part 2 - Mutex Locking*****\n");
    clock_gettime(CLOCK_MONOTONIC, &time_start); // log start time for relative time calculations
    printf("Start time: %ld.%09ld sec\n", time_start.tv_sec, time_start.tv_nsec);
    printf("Relative time (time since start) is used with dummy value generation for each verification.\n\n");

    printf("Begin setting up scheduler and threads...\n");
    //*********CONFIGURE SCHEDULER, CHECK CORE MAIN IS ON (NOT PINNED BUT WORKERS AND SEQUENCER ARE ON 3)*/
    /*call set_scheduler to make process (file) FIFO highest pri, configures attributes
    for FIFO, CPU3, max pri to use in thread creation, can decrement pri on thread assignment*/
    set_scheduler();
    CPU_ZERO(&cpuset); // just clears the var for use below, no real effect yet
    // get affinity set for main thread
    main_thread = pthread_self();
    // Check the affinity mask assigned to the main thread and print - since unaltered should be all
    rc = pthread_getaffinity_np(main_thread, sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
        perror("pthread_getaffinity_np");
    else
    {
        printf("main thread running on CPU=%d, CPUs =", sched_getcpu());

        for (j = 0; j < CPU_SETSIZE; j++)
            if (CPU_ISSET(j, &cpuset))
                printf(" %d", j);

        printf("\n");
    }
    // ensure main is highest pri - done with set_scheduler no need to do again

    //*********CONFIGURE MUTEX ATTRIBUTES, INIT MUTEX*******/
    printf("Initializing mutex...\n");
    if (pthread_mutexattr_init(&data_mutex_attr))
    {
        perror("mutex attr init failed");
        exit(-1);
    }
    if (pthread_mutexattr_setprotocol(&data_mutex_attr, PTHREAD_PRIO_INHERIT))
    {
        perror("mutex attr set protocol failed");
        exit(-1);
    }
    if (pthread_mutexattr_settype(&data_mutex_attr, PTHREAD_MUTEX_NORMAL))
    {
        perror("mutex attr set type failed");
        exit(-1);
    }
    if (pthread_mutex_init(&data_mutex, &data_mutex_attr))
    {
        perror("mutex init failed");
        exit(-1);
    }

    //*********CREATE THREADS*******/
    /* create the update thread and read thread with fifo_sched_attr which has cpu3 pin, fifo, max prio attributes, but decrement pri for each thread */

    //decrement priority and begin 1Hz update thread
    fifo_param.sched_priority -= 1; // update thread will have pri 98
    pthread_attr_setschedparam(&fifo_sched_attr, &fifo_param); //accept priority into attributre
    pthread_create(&update_thread,   // pointer to thread descriptor
                   &fifo_sched_attr, // use FIFO RT max priority attributes
                   updateThread,     // thread function entry point
                   (void *)0         // parameters to pass in
    );
    printf("Update thread created with priority %d\n", fifo_param.sched_priority);
    
    //decrement priority and begin 0.1Hz read thread
    fifo_param.sched_priority -= 1; // update thread will have pri 97
    pthread_attr_setschedparam(&fifo_sched_attr, &fifo_param); //accept priority into attributre
    pthread_create(&read_thread,     // pointer to thread descriptor
                   &fifo_sched_attr, // use FIFO RT max priority attributes
                   readThread,       // thread function entry point
                   (void *)0         // parameters to pass in
    );
    printf("Read thread created with priority %d\n\n", fifo_param.sched_priority);
    
    // allow sequencer to complete before main can continue - block main
    pthread_join(update_thread, NULL);
    pthread_join(read_thread, NULL);
    printf("All threads complete and joined, printing summary of reads and cleaning mutex and attributes object.\n");

    //*********PRINT SUMMARY*******/
    printf("******* READ SUMMARY *******\n");
    for(i=0; i<MAX_READS; i++){
        printf("Read values from int seconds of relative time\n{\n lat=%f\n lon=%f\n alt=%f\n roll=%f\n pitch=%f\n yaw=%f\n write_timestamp=%ld.%09ld\n seq_count=%d\n}\n\n",
               read_summary[i].lat,
               read_summary[i].lon,
               read_summary[i].alt,
               read_summary[i].roll,
               read_summary[i].pitch,
               read_summary[i].yaw,
               read_summary[i].timestamp.tv_sec,
               read_summary[i].timestamp.tv_nsec,
               read_summary[i].sequence_count);
        }

    //*********CLEANUP MUTEX*******/
    pthread_mutex_destroy(&data_mutex);
    pthread_mutexattr_destroy(&data_mutex_attr);
    printf("\n****** EXERCISE 3 SCRIPT COMPLETE ******\n");
}