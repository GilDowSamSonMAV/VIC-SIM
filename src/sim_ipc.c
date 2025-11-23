#include "sim_ipc.h"

#include <unistd.h>
#include <errno.h>

// Phase_Migration: helper to read exactly n bytes from an fd
ssize_t read_full(int fd, void *buf, size_t n)
{
    size_t total = 0;
    char *p = (char *)buf;

    while (total < n) {
        ssize_t r = read(fd, p + total, n - total);
        if (r < 0) {
            if (errno == EINTR) { // Case where it was blocked by a signal
                continue;
            }
            return -1; 
        }
        if (r == 0) {
            break;
        }
        total += (size_t)r;
    }

    return (ssize_t)total;
}

// Phase_Migration: helper to write exactly n bytes to an fd
ssize_t write_full(int fd, const void *buf, size_t n)
{
    size_t total = 0;
    const char *p = (const char *)buf;

    while (total < n) {
        ssize_t w = write(fd, p + total, n - total);
        if (w < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        if (w == 0) {
            break;
        }
        total += (size_t)w;
    }

    return (total == n) ? (ssize_t)total : -1;
}
