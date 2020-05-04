
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include "nrfx_uarte.h"

extern nrfx_uarte_t uart0;

__attribute__((used))
int _write_r(struct _reent *r, int file, char *data, int len)
{
	(void)r;
	(void)file;
	(void)data;
	(void)len;

	return -ENODEV;
}

__attribute__((used))
int _read_r(struct _reent *r, int file, char *data, int len)
{
	(void)r;
	(void)file;
	(void)data;
	(void)len;

	return -ENODEV;
}

