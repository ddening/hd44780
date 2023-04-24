/*************************************************************************
* Title		: suite.c
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
@file suite.c
@author Dimitri Dening
@date 13.02.2022
*/
#include <stdio.h>
#include "suite.h"
#include "uart.h"

static void test_report_failure(const struct test_case* test, const char* stage, int result) {
	uart_put("%s '%s' %s %d", "Test", test->name, "failed:", result);
}

static int test_call(int (*func)(const struct test_case*), const struct test_case* test) {
	int ret = 0;

	if (!func) {
		return TEST_PASS;
	}

	ret = func(test);

	if (ret == TEST_ERROR) return TEST_ERROR;
	if (ret == TEST_FAIL)  return TEST_FAIL;
	
	return TEST_PASS;
}

static int test_case_run(const struct test_case* test) {
	int result;
	
	uart_put("%s '%s'", "Running test:", test->name);

	result = test_call(test->run, test);
	if (result) {
		test_report_failure(test, "test", result);
	}

	return result;
}

int test_suite_run(const struct test_suite* suite) {
	unsigned int nr_failures = 0;
	unsigned int nr_errors = 0;
	unsigned int i;
	int          ret;

	uart_put("%s '%s' %s", "Running test suite", suite->name, "...");

	for (i = 0; i < suite->nr_tests; i++) {
		const struct test_case* test;

		test = suite->tests[i];
		ret = test_case_run(test);
		if (ret < TEST_PASS) {
			nr_errors++;
		}
		else if (ret > TEST_PASS) {
			nr_failures++;
		}
	}

	uart_put("%s '%s' %s %u %s %u %s %u %s", "Test suite", suite->name, "complete:", suite->nr_tests, "tests,", nr_failures, "failures,", nr_errors, "errors\r\n");

	return nr_errors + nr_failures;
}