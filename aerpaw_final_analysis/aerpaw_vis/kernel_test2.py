
'''

'''

def __call__(self, X, Y=None, eval_gradient=False):
    if Y is None:
        Y = X

    # Distance differences
    d_diff = X[:, 0][:, np.newaxis] - Y[:, 0][np.newaxis, :]
    # Frequency differences
    f_diff = X[:, 1][:, np.newaxis] - Y[:, 1][np.newaxis, :]
    
    # Length scale based on distance and frequency
    length_scale = self.ref_d * np.exp(-self.b * X[:, 0]) * np.exp(-self.c * X[:, 1])
    length_scale_diff = d_diff**2 + f_diff**2
    
    K = np.exp(-0.5 * length_scale_diff / length_scale[:, np.newaxis]**2)
    
    if eval_gradient:
        # Compute gradient w.r.t. b, c, and f (for hyperparameter tuning)
        dK_db = K * length_scale_diff * X[:, 0][:, np.newaxis] * length_scale[:, np.newaxis]
        dK_dc = K * length_scale_diff * X[:, 1][:, np.newaxis] * length_scale[:, np.newaxis]
        dK_df = K * f_diff**2 / self.f

        return K, np.dstack((dK_db, dK_dc, dK_df))
    
    return K
