import pytest
import numpy as np
import toppra
import toppra.constraint as constraint


@pytest.fixture(params=[(0, 0)])
def basic_constraints(request):
    """ Return a set of relatively simple constraints.
    """
    dtype_a, dtype_ra = request.param
    vlims = np.array([[-1, 1], [-1, 2], [-1, 4], [-3, 4], [-2, 4], [-3, 4], [-2, 5]],
                     dtype=float) * 10
    alims = np.array([[-1, 1], [-1, 2], [-1, 4], [-3, 4], [-2, 4], [-3, 4], [-2, 5]],
                     dtype=float) * 10

    vel_cnst = constraint.JointVelocityConstraint(vlims)
    accl_cnst = constraint.JointAccelerationConstraint(alims, dtype_a)
    robust_accl_cnst = constraint.RobustLinearConstraint(
        accl_cnst, [1e-4, 1e-4, 5e-4], dtype_ra)
    yield vel_cnst, accl_cnst, robust_accl_cnst


@pytest.fixture(params=["spline", "poly"])
def basic_path(request):
    """ Return a generic path.
    """
    if request.param == "spline":
        np.random.seed(1)
        path = toppra.SplineInterpolator(np.linspace(0, 1, 5), np.random.randn(5, 7))
    elif request.param == "poly":
        np.random.seed(1)
        coeffs = np.random.randn(7, 3)  # 7 random quadratic equations
        path = toppra.PolynomialPath(coeffs)
    yield path
