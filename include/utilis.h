/*
 * Zynq-SJSU IP Interface Header Files: utilis.h
 *
 * This header file contains some useful marcos and typedefs
 * that are mostly copied from header files of Linux Kernel.
 */

typedef _Bool bool;
typedef signed char s8;
typedef unsigned char u8;
typedef signed short s16;
typedef unsigned short u16;
typedef signed int s32;
typedef unsigned int u32;
typedef signed long long s64;
typedef unsigned long long u64;

#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))

/* Since we suppose it is only used for Zynq Platform, use 4KB page size */
#define PAGE_SIZE	(1UL << 12)
#define PAGE_ALIGN(x)	((x + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))

#define pr_debug(fmt, ...) \
	do { \
		if (DEBUG) \
			printf(fmt, ##__VA_ARGS__); \
	} while (0)
