from waldo_commander.main import _ping_state_after_sample


def test_connected_page_ignores_two_transient_ping_failures() -> None:
    ok, failures = _ping_state_after_sample(last_ok=True, failures=0, sample_ok=False)
    assert (ok, failures) == (True, 1)
    ok, failures = _ping_state_after_sample(last_ok=ok, failures=failures, sample_ok=False)
    assert (ok, failures) == (True, 2)
    ok, failures = _ping_state_after_sample(last_ok=ok, failures=failures, sample_ok=False)
    assert (ok, failures) == (False, 3)


def test_success_resets_ping_failure_hysteresis() -> None:
    assert _ping_state_after_sample(last_ok=True, failures=2, sample_ok=True) == (True, 0)


def test_disconnected_startup_does_not_claim_hardware() -> None:
    assert _ping_state_after_sample(last_ok=False, failures=0, sample_ok=False) == (False, 1)


def test_running_program_keeps_last_confirmed_connection_on_ping_failure() -> None:
    assert _ping_state_after_sample(
        last_ok=True,
        failures=2,
        sample_ok=False,
        suppress_failure=True,
    ) == (True, 0)
