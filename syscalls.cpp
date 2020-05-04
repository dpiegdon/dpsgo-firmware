
#include <unistd.h>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <algorithm>

#include "syscalls.h"

static Uarte * stdio_uarte = NULL;

static embedded_drivers::Ssd1306I2cDisplay * stdio_ssd1306 = NULL;


void stdio_via_uarte(Uarte * new_uarte)
{
	printf("STDIO:=UARTE\r\n");
	stdio_uarte = new_uarte;
	printf("STDIO==UARTE\r\n");
}

void stdio_via_ssd1306(embedded_drivers::Ssd1306I2cDisplay * new_ssd1306)
{
	printf("STDIO:=SSD1306\r\n");
	stdio_ssd1306 = new_ssd1306;
	printf("STDIO==SSD1306\r\n");
}


extern "C"
__attribute__((used))
int _write_r(struct _reent *r, int file, char *data, int len)
{
	(void)r;

	if(len < 0)
		return -EINVAL;

	if(!nrfx_is_in_ram(data)) {
		// EasyDMA requires all payload in RAM.
		// So make sure it really is there.
		unsigned todo = len;
		while(todo) {
			char ramCopy[16];
			unsigned doNow = std::min(todo, sizeof(ramCopy));
			memcpy(ramCopy, data, doNow);
			int ret = _write_r(r, file, ramCopy, doNow);
			if(ret < 0)
				return ret;
			data += doNow;
			todo -= doNow;
		}

		return len;
	} else {
		if(STDOUT_FILENO != file)
			return -ENODEV;

		int ret = -ENODEV;

		// send to *all* found devices

		if(stdio_uarte)
			ret = stdio_uarte->Write(data, len);

		if(stdio_ssd1306)
			ret = stdio_ssd1306->Write(data, len);

		return ret;
	}
}

extern "C"
__attribute__((used))
int _read_r(struct _reent *r, int file, char *data, int len)
{
	(void)r;

	if(STDIN_FILENO != file)
		return -ENODEV;

	if(stdio_uarte)
		return stdio_uarte->Read(data, len);

	return -ENODEV;
}

