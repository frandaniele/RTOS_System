#define putchar(c) c

#include <stdarg.h>

#define PAD_RIGHT 1
#define PAD_ZERO 2
/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static void printchar(char **str, int c, char *buflimit);

static int prints(char **out, const char *string, int width, int pad, char *buflimit);

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase, char *buflimit);

static int tiny_print( char **out, const char *format, va_list args, unsigned int buflen );

int sprintf(char *out, const char *format, ...);

int	write( int i, char* c, int n);