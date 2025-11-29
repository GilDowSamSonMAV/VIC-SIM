#include "sim_log.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>

static FILE *log_fp = NULL;
static int   log_owns_fp = 0;  // 1 if we opened a real file, 0 if using stderr

// Internal helper: get ISO-like timestamp "YYYY-MM-DD HH:MM:SS"
static void log_get_timestamp(char *buf, size_t buf_size)
{
    if (!buf || buf_size == 0) return;

    time_t now = time(NULL);
    struct tm tm_now;

    // Thread-safe variant where available; falls back otherwise
#if defined(_POSIX_THREAD_SAFE_FUNCTIONS)
    localtime_r(&now, &tm_now);
#else
    struct tm *tmp = localtime(&now);
    if (!tmp) {
        buf[0] = '\0';
        return;
    }
    tm_now = *tmp;
#endif

    if (strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", &tm_now) == 0) {
        buf[0] = '\0';
    }
}

void sim_log_init(const char *process_name)
{
    if (log_fp) {
        // Already initialized
        return;
    }

    if (!process_name || process_name[0] == '\0') {
        // Fallback to stderr if no name is provided
        log_fp = stderr;
        log_owns_fp = 0;
        return;
    }

    // Log directory is in project root: bin/log/
    // Our binaries run from build/src/, so we need ../../bin/log/
    char path[256];
    snprintf(path, sizeof(path), "../../bin/log/%s.log", process_name);

    FILE *fp = fopen(path, "a");
    if (!fp) {
        // Fallback to stderr if we can't open the file
        log_fp = stderr;
        log_owns_fp = 0;
        fprintf(stderr,
                "sim_log: could not open '%s' for writing, falling back to stderr\n",
                path);
        return;
    }

    // Line-buffered logging is usually good for log files
    setvbuf(fp, NULL, _IOLBF, 0);

    log_fp = fp;
    log_owns_fp = 1;

    // Write a small header line for each process start
    char ts[32];
    log_get_timestamp(ts, sizeof(ts));
    fprintf(log_fp, "[%s] [INFO] --- %s started ---\n",
            (ts[0] ? ts : "??????????"), process_name);
}

void sim_log_info(const char *fmt, ...)
{
    if (!log_fp) {
        // In case someone forgot init, fall back silently to stderr
        log_fp = stderr;
        log_owns_fp = 0;
    }

    char ts[32];
    log_get_timestamp(ts, sizeof(ts));

    fprintf(log_fp, "[%s] [INFO] ", (ts[0] ? ts : "??????????"));

    va_list ap;
    va_start(ap, fmt);
    vfprintf(log_fp, fmt, ap);
    va_end(ap);

    fputc('\n', log_fp);
    fflush(log_fp);
}

void sim_log_close(void)
{
    if (log_fp && log_owns_fp) {
        fclose(log_fp);
    }
    log_fp = NULL;
    log_owns_fp = 0;
}
