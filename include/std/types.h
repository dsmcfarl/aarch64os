#pragma once

// Integer types.
typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;
typedef long int64;
typedef unsigned long uint64;

// Boolean type.
typedef int8 bool;

// Floating-point types.
typedef float float32;
typedef double float64;

// Miscellaneous types.
typedef uint8 byte;
typedef int64 size;
typedef int64 error;
typedef int64 size_or_error;
extern error const ERROR_NONE;	// 0
extern error const ERROR;	// 1
