import numpy as np
# import pdb

class Gaussian:
    def __init__(self, _mu, _var):
        self.mu = _mu
        self.var = _var

    @staticmethod
    def diagonal(mu_vect, variance_vect):
        if(isinstance(mu_vect, list) and isinstance(variance_vect, list)):
            assert(len(mu_vect) == len(variance_vect))
            return Gaussian( np.array(mu_vect), np.diag(variance_vect) )
        else:
            raise NotImplementedError('Use lists for mu and var vector args!')

class KalmanFilter:
    def __init__(self, _A, _B, _C, _D, _w, _v, _x, _K):
        self.A = _A
        self.B = _B,
        self.C = _C
        self.D = _D
        
        self.w = _w
        self.v = _v
        self.x = _x

        self.K = _K

    def predict(self, u):
        tx_mu = np.dot(self.A, self.x.mu) + np.dot(self.B, u)
        tx_var = np.dot(self.A, np.dot(self.x.var, self.A.T) ) + self.w.var

        self.x.mu, self.x.var = tx_mu, tx_var

    def correct(self, y):
        yhat_mu = np.dot(self.C, self.x.mu)
        yhat_var = self.v.var + np.dot(self.C, np.dot( self.x.var, self.C.T ))

        self.K = np.dot( self.x.var, np.dot( self.C.T, np.linalg.inv( yhat_var ) ) )

        self.x.mu = self.x.mu + np.dot( self.K, (y-yhat_mu) )
        self.x.var = self.x.var - np.dot( self.K, np.dot( self.C, self.x.var ) )
        # self.x.var = self.x.var - np.dot( self.K, np.dot( yhat_var, self.K.T ) )

    def get_state(self):
        return self.x.mu, self.x.var

class BallTracker(KalmanFilter):
    
    def __init__(self, _X, _Y, _R):
        self.A = np.array([  \
            [1, 0, 0, 1, 0, 0], \
            [0, 1, 0, 0, 1, 0], \
            [0, 0, 1, 0, 0, 1], \
            [0, 0, 0, 1, 0, 0], \
            [0, 0, 0, 0, 1, 0], \
            [0, 0, 0, 0, 0, 1], \
        ])

        self.C = np.array([  \
            [1, 0, 0, 0, 0, 0],   \
            [0, 1, 0, 0, 0, 0],   \
            [0, 0, 1, 0, 0, 0],   \
            [0, 0, 0, 1, 0, 0],   \
            [0, 0, 0, 0, 1, 0],   \
            [0, 0, 0, 0, 0, 1],   \
        ])

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0, 0, 0, 0, 0], [1e-4, 1e-4, 1e-5, 1e-4, 1e-4, 1e-6] )
        self.v = Gaussian.diagonal( [0, 0, 0, 0, 0, 0], [5e-5, 5e-5, 1e-2, 1e-4, 1e-4, 1e-3] )

        self.x = Gaussian.diagonal( [_X, _Y, _R, 0, 0, 0], [1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-4] )

        self.yold = [_X, _Y, _R]

    def correct(self, y):
        ty = np.append(y, [ y[ix]-self.yold[ix] for ix in range(len(y)) ] )
        # pdb.set_trace()
        KalmanFilter.correct(self, ty)
        self.yold = y


class BearingTracker(KalmanFilter):
    
    def __init__(self, _dist, _theta):
        self.A = np.array([  \
            [1, 0, 1, 0], \
            [0, 1, 0, 1], \
            [0, 0, 1, 0], \
            [0, 0, 0, 1], \
        ])

        self.C = np.array([  \
            [1, 0, 0, 0],   \
            [0, 1, 0, 0],   \
            [0, 0, 1, 0],   \
            [0, 0, 0, 1],   \
        ])

        self.B = np.array([0])
        self.D = np.array([0])

        self.w = Gaussian.diagonal( [0, 0, 0, 0], [1e-5, 1e-4, 1e-6, 1e-4] )
        self.v = Gaussian.diagonal( [0, 0, 0, 0], [1e-2, 5e-5, 1e-3, 1e-4] )

        self.x = Gaussian.diagonal( [_dist, _theta, 0, 0], [1e-3, 1e-3, 1e-5, 1e-4] )

        self.yold = [_dist, _theta]

    def correct(self, y):
        ty = np.append(y, [ y[ix]-self.yold[ix] for ix in range(len(y)) ] )
        # pdb.set_trace()
        KalmanFilter.correct(self, ty)
        self.yold = y