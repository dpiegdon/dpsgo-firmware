
#include "dpsgo.h"

TaskHandle_t logicManager = NULL;
void logicManagerTask(void * ignored)
{
	(void)ignored;

	while(1) { };

	// wait for all threads to do a basic hardware init

	// enable all power rails

	// upload FPGA bitstream

	// program frequency synthesizer

	// enter operating_state:
	//   wait for some message.
	//   if FPGA frequency capture message:
	//     calculate frequency difference
	//     use some kind of control mechanism (PID?)
	//     set new DAC value
	//     reset FPGA counter
	//   if FPGA interface message:
	//     adapt UI? or delegate UI to i2c thread?
	//   if message from UART:
	//     receive and parse GPS message
	//     set according flags (FIX/NO FIX) and time
	//     update screen
	//   if other message (interrupt, GPIO):
	//     check alerts et al
	//   goto operating_state
}

