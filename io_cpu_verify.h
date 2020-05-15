/*
 *
 * tests specific to the nrf52 CPU
 *
 */
#ifndef nrf52_verify_H_
#define nrf52_verify_H_
#ifdef IMPLEMENT_VERIFY_IO_CPU

UNIT_SETUP(setup_io_cpu_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_io_cpu_unit_test) {
}

static void
io_cpu_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		0
	};
	unit->name = "io cpu";
	unit->description = "io cpu unit test";
	unit->tests = tests;
	unit->setup = setup_io_cpu_unit_test;
	unit->teardown = teardown_io_cpu_unit_test;
}

void
run_ut_io_cpu (V_runner_t *runner) {
	static const unit_test_t test_set[] = {
		io_cpu_unit_test,
		0
	};
	V_run_unit_tests(runner,test_set);
}

#endif /* IMPLEMENT_VERIFY_IO_CPU */
#endif