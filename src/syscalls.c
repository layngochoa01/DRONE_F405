#include <sys/stat.h>
#include "uart5.h"

extern char _heap_end;
static char *heap_ptr = 0;

void *_sbrk(int incr)
{
    if (heap_ptr == 0) heap_ptr = &_heap_end;
    char *prev = heap_ptr;
    heap_ptr += incr;
    return (void *)prev;
}


int _write(int fd, char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) UART5_WriteChar(ptr[i]);
    return len;
}

int _read(int fd, char *ptr, int len)      { (void)fd; (void)ptr; (void)len; return 0; }
int _close(int fd)                         { (void)fd; return -1; }
int _fstat(int fd, struct stat *st)        { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                        { (void)fd; return 1; }
int _lseek(int fd, int ptr, int dir)       { (void)fd; (void)ptr; (void)dir; return 0; }
void _exit(int status)                     { (void)status; while(1) {} }
int _kill(int pid, int sig)                { (void)pid; (void)sig; return -1; }
int _getpid(void)                          { return 1; }