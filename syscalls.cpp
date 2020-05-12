
#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <algorithm>

#include "dpsgo.h"

extern "C"
__attribute__((used))
int _write_r(struct _reent *r, int file, const void *data, size_t len)
{
	(void)r;

	if((STDOUT_FILENO != file) || (NULL == console))
		return -ENODEV;

	int ret = -EINVAL;

	if(NULL != console) {
		taskENTER_CRITICAL();
		ret = xStreamBufferSend(console, data, len, 0);
		taskEXIT_CRITICAL();
	};

	return ret;
}

extern "C"
__attribute__((used))
int _read_r(struct _reent *r, int file, void *data, size_t len)
{
	(void)r;
	(void)file;
	(void)data;
	(void)len;

	if(STDIN_FILENO != file)
		return -ENODEV;

	return -ENODEV;
}

