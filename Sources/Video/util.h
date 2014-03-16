#ifndef _UTIL_H_
#define _UTIL_H_

extern "C" {

/*!
 * Sleeps for a given number of milliseconds.
 * \param msecs The number of milliseconds to sleep.
 * \blocks
 * \ingroup util
 */
void msleep(long msecs);

/*!
 * Get the current system time in milliseconds.
 * \return the number of milliseconds since the UNIX epoch (January 1st, 1970)
 * \ingroup util
 */
unsigned long systime();

/*!
 * Get the current system time in seconds.
 * \return the number of seconds since the UNIX epoch (January 1st, 1970)
 * \ingroup util
 */
double seconds();

}

#endif
