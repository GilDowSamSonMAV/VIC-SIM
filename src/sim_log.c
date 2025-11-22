// If we stick to log logic as its done now it will be added here 

#include "sim_log.h"
#include <stdio.h>
#include <stdarg.h>

static FILE *log_fp = NULL;

void sim_log_init(const char *process_name) {
    (void)process_name; 
    log_fp = stderr;
}

void sim_log_info(const char *fmt, ...) {
    if (!log_fp) return;
    va_list ap;
    va_start(ap, fmt);
    vfprintf(log_fp, fmt, ap);
    fprintf(log_fp, "\n");
    va_end(ap);
}

void sim_log_close(void) {
    
}
