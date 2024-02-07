import numpy as np
from sklearn.gaussian_process.kernels import Kernel, Hyperparameter, _check_length_scale

class NonStationaryRBF(Kernel):
    """ Non-stationary RBF kernel with length scale l(x) = a * x + b. """

    def __init__(self, a=1.0, a_bounds=(1e-5, 1e5), b=1.0, b_bounds=(1e-5, 1e5)):
        self.a = a
        self.b = b
        self.a_bounds = a_bounds
        self.b_bounds = b_bounds

    @property
    def hyperparameter_a(self):
        return Hyperparameter("a", "numeric", self.a_bounds)

    @property
    def hyperparameter_b(self):
        return Hyperparameter("b", "numeric", self.b_bounds)

    def __call__(self, X, Y=None, eval_gradient=False):
        if Y is None:
            Y = X

        a = _check_length_scale(X, self.a)
        b = _check_length_scale(X, self.b)
        length_scale = a * X + b
        
        length_scale_diff = (X[:, np.newaxis] - Y[np.newaxis, :]) / length_scale[:, np.newaxis]
        K = np.exp(-0.5 * np.sum(length_scale_diff ** 2, axis=-1))
        
        if eval_gradient:
            # Compute gradient w.r.t. a and b (for hyperparameter tuning)
            dK_da = K * np.sum(length_scale_diff ** 2, axis=-1) * X[:, np.newaxis]
            dK_db = K * np.sum(length_scale_diff ** 2, axis=-1)
            return K, np.dstack((dK_da, dK_db))
        
        return K

    def diag(self, X):
        return np.ones(X.shape[0])

    def is_stationary(self):
        return False

# Example Usage
from sklearn.gaussian_process import GaussianProcessRegressor

X = np.array([[1.], [2.], [3.]])
y = np.array([2., 3., 4.])

kernel = NonStationaryRBF(a=0.5, b=1.0)
gp = GaussianProcessRegressor(kernel=kernel).fit(X, y)
