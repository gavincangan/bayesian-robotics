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

        # self.mahalonobis_threshold = np.inf #1.0

    def predict(self, u, w=None):
        if(w is None):
            w_var = self.w.var

        tx_mu = np.dot(self.A, self.x.mu) + np.dot(self.B, u)
        tx_var = np.dot(self.A, np.dot(self.x.var, self.A.T) ) + w_var

        self.x.mu, self.x.var = tx_mu, tx_var

    def correct(self, y, v_var=None):

        # if v is None:
        # #     v_var = self.v.var
        # # elif isinstance(v, np.ndarray):
        # #     v_var = v
        # # elif isinstance(v, Gaussian):
        # #     v_var = v.var
        # # else:
        #     raise NotImplementedError

        innov_mu = y - np.dot(self.C, self.x.mu)
        innov_var = np.linalg.inv( v_var + np.dot(self.C, np.dot( self.x.var, self.C.T )) )

        # mahalonobis_dist = np.dot(innov_mu, np.dot(innov_var, innov_mu ) )
        
        # if(mahalonobis_dist <= self.mahalonobis_threshold):

        self.K = np.dot( self.x.var, np.dot( self.C.T, innov_var ) )

        self.x.mu = self.x.mu + np.dot( self.K, innov_mu )
        self.x.var = self.x.var - np.dot( self.K, np.dot( self.C, self.x.var ) )

        # else:
        #     print('M dist: {}\tRejecting measurement: {}'.format(mahalonobis_dist, y))
        # self.x.var = self.x.var - np.dot( self.K, np.dot( yhat_var, self.K.T ) )

    def get_state(self):
        return self.x.mu, self.x.var
