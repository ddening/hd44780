/*************************************************************************
* Title		: suite.h
* Author	: Dimitri Dening
* Created	: 13.02.2022 19:30:50
* Software	: Microchip Studio V7
* Hardware	: Atmega1284P
*
* This is a slighty modified version of Microchip Studios' SPI Unit Test.
* Following the license from Microchip used in the original file.
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products.
* It is your responsibility to comply with third party license terms applicable
* to your use of third party software (including open source software) that
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
*************************************************************************/

/**
@file suite.h
@author Dimitri Dening
@date 13.02.2022
*/

#ifndef TEST_SUITE_H_INCLUDED
#define TEST_SUITE_H_INCLUDED

enum test_status {
	TEST_ERROR = -1,
	TEST_PASS = 0,  
	TEST_FAIL = 1,
};

struct test_case {
	void (*setup)(const struct test_case* test);
	int (*run)(const struct test_case* test);
	void (*cleanup)(const struct test_case* test);
	const char* name;
};

struct test_suite {
	unsigned int nr_tests;				  // The number of tests in this suite
	const struct test_case* const* tests; // Array of pointers to the test cases
	const char* name;					  // The name of the test suite
};

#define DEFINE_TEST_CASE(_sym, _setup, _run, _cleanup, _name)       \
	static const char _test_str_##_sym[] = _name;                   \
	static const struct test_case _sym = {                          \
		.setup   = _setup,                                          \
		.run     = _run,                                            \
		.cleanup = _cleanup,                                        \
		.name    = _test_str_##_sym,                                \
	}

#define DEFINE_TEST_ARRAY(_sym)                                     \
	const struct test_case *const _sym[]

#define DEFINE_TEST_SUITE(_sym, _test_array, _name)                    \
	static const char _test_str_##_sym[] = _name;                      \
	const struct test_suite _sym = {                                   \
		.nr_tests = ARRAY_LEN(_test_array),                            \
		.tests    = _test_array,                                       \
		.name     = _test_str_##_sym,                                  \
	}

#ifndef ARRAY_LEN
# define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#endif

int test_suite_run(const struct test_suite* suite);

#endif /* TEST_SUITE_H_INCLUDED */