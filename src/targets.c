// Implement targets here
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>


typedef struct {
    float t_x;
    float t_y;
    int active; 
    //we declare a nested structure so we can save the time it was generated
    struct timespec time_created;
}targets;

typedef struct {
    int window_height;
    int window_length;
}dimensions;

static void generate_targets(int h, int w, int number){

    for (int i = 0; i < number; i++){
        targets[i].t_x = 0;
    }
}


int main(){

    dimensions d; 
    struct timeval tv;
    // we declare the wait for the reading
    tv.tv_sec = 20; // 20s timeout
    tv.tv_usec = 0;   

    // variable delcaration
    int target_number = 0;

    // now we declare the targets
    targets t[target_number];


    // We get the number of targets read by the BB from the parameter file
    int f = read(STDERR_FILENO, &target_number, sizeof(int));
    if (f < 0){
        perror("T: Issue with the first reading");
    }

    

    // the initial read for the window size (we make it blocking bc it doesn't make sense
    // to continue without having it)
    f = read(STDERR_FILENO, &d, sizeof(d));
    if (f < 0){
        perror("T: Issue with the second reading");
    }

    // now we make it nonblocking 
    int fd = STDIN_FILENO;
    fd_set rfd;
    FD_ZERO(&rfd);
    FD_SET(fd, &rfd);

    while(1){
        // we generate the targets
        generate_targets(d.window_height, d.window_length, target_number);


        // we check if there is a new window size 
        int is_data = select(fd+1, &rfd, NULL, NULL, &tv);
        if (is_data > 0){
            // the window has been resized
            f = read(STDIN_FILENO, &d, sizeof(d));
            if (f < 0){
                perror("T: Issue with window resize reading");
            }
        }

        // if there isn't a new window we generate a new batch of 
        // targets randomly after 20 seconds

        
        
    }

    return 0;
}