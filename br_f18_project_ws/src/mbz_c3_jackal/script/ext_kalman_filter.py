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

class ExtKalmanFilter:
    def __init__(self, _w, _v, _x, _K):
        self.w = _w
        self.v = _v
        self.x = _x

        self.K = _K
        self.u = None
        
        self.mahalonobis_threshold = np.inf #1.0

    def nextState(self):
        raise NotImplementedError

    def jacobianA(self):
        raise NotImplementedError

    def expMeasurement(self, DATATYPE=None):
        raise NotImplementedError

    def jacobianC(self, DATATYPE=None):
        raise NotImplementedError

    def predict(self):
        # if(w is None):
        #     w_var = self.w.var

        self.nextState()

        jacA = self.jacobianA()
        self.x.var = np.dot(jacA, np.dot(self.x.var, jacA.T) ) + self.w.var

    def correct(self, y, v=None, DATATYPE=None):

        if v is None:
            v_var = self.v.var
        elif isinstance(v, np.ndarray):
            v_var = v
        elif isinstance(v, Gaussian):
            v_var = v.var
        else:
            raise NotImplementedError

        y_hat = self.expMeasurement(DATATYPE)
        jacC = self.jacobianC(DATATYPE)

        innov_mu = y - y_hat
        innov_var_inv = v_var + np.dot(jacC, np.dot( self.x.var, jacC.T ))
        if abs( np.linalg.det(innov_var_inv) ) < 1e-10:
            return

        innov_var = np.linalg.inv( innov_var_inv )

        # mahalonobis_dist = np.dot(innov_mu, np.dot(innov_var, innov_mu ) )
        # if(mahalonobis_dist <= self.mahalonobis_threshold):

        self.K = np.dot( self.x.var, np.dot( jacC.T, innov_var ) )

        self.x.mu = self.x.mu + np.dot( self.K, innov_mu )
        self.x.var = self.x.var - np.dot( self.K, np.dot( jacC, self.x.var ) )

        # else:
        #     print('M dist: {}\tRejecting measurement: {}'.format(mahalonobis_dist, y))
        # self.x.var = self.x.var - np.dot( self.K, np.dot( yhat_var, self.K.T ) )

    def get_state(self):
        return self.x.mu, self.x.var