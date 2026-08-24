/**
 * @file test_runner.h
 * @brief Test suite runner.
 */

#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

/**
 * @brief Initialize the test environment.
 */
void test_runner_init(void);

/**
 * @brief Run a specific test suite interactively based on serial input.
 */
void test_runner_update(void);

#endif /* TEST_RUNNER_H */
