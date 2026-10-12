import numpy as np
import pytest
from ur_vr_teleop.comparison import simulate, metrics


@pytest.mark.parametrize('scenario', ['hold', 'reversal', 'small'])
@pytest.mark.parametrize('delay,lag', [(0., 0.), (.08, .1), (.15, .2)])
def test_delayed_plant_stop_and_limits(scenario, delay, lag):
    results = {}
    for mode in ('position', 'velocity'):
        rows = simulate(mode, scenario, delay, lag)
        results[mode] = metrics(rows)
        commands = np.array([r['command'] for r in rows])
        assert np.all(np.isfinite(commands))
        assert np.max(np.abs(commands)) <= .1+1e-12
        assert np.max(np.abs(np.diff(commands))) <= .0025+1e-12
        assert abs(rows[-1]['velocity']) < .001
        assert results[mode]['stop_latency_s'] is not None
    # Test the defining architectural property, not a blanket performance claim.
    rows = simulate('velocity', scenario, delay, lag)
    assert abs(rows[-1]['command']) < 1e-10
