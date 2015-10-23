#ifndef MACROS_H
#define MACROS_H

/*
 * Generally useful macros live here.
 */

// Produces a 'size of array is negative' compile error when the assert fails
#define STATIC_ASSERT(_x)  ((void)sizeof(char[1 - 2*!(_x)]))

#endif // MACROS_H
