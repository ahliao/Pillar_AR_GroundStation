#include "util.h"
#include <sys/time.h>

void msleep(long msecs)
{

}

unsigned long systime()
{
	timeval t;
	gettimeofday(&t, 0);
	return ((unsigned long) t.tv_sec) * 1000L + t.tv_usec / 1000L;
}

double seconds()
{
	return systime() / 1000.0;
}
