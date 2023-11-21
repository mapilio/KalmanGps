from math import *
import numpy as np
import pandas as pd

# Notation used coming from: https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
def prediction(X_hat_t_1, P_t_1, F_t, B_t, U_t, Q_t):
    X_hat_t = F_t.dot(X_hat_t_1) + (B_t.dot(U_t))
    P_t = np.diag(np.diag(F_t.dot(P_t_1).dot(F_t.transpose()))) + Q_t
    return X_hat_t, P_t
    
def update(X_hat_t, P_t, Z_t, R_t, H_t):
    K_prime = P_t.dot(H_t.transpose()).dot(np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
    X_t = X_hat_t + K_prime.dot(Z_t - H_t.dot(X_hat_t))
    P_t = P_t - K_prime.dot(H_t).dot(P_t)
    return X_t, P_t

def rotaitonMatrix(heading, attitude, bank):
    '''
    :returns: rotation array in numpy format
    [m00 m01 m02]
    [m10 m11 m12]
    [m20 m21 m22]
    '''
    ch = cos(heading)
    sh = sin(heading)
    ca = cos(attitude)
    sa = sin(attitude)
    cb = cos(bank)
    sb = sin(bank)
    m00 = ch * ca
    m01 = sh * sb - ch * sa * cb
    m02 = ch * sa * sb + sh * cb
    m10 = sa
    m11 = ca * cb
    m12 = -ca * sb
    m20 = -sh * ca
    m21 = sh * sa * cb + ch * sb
    m22 = -sh * sa * sb + ch * cb
    return np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]])
    
def getDistance(lat1, lon1, lat2, lon2):
    '''
    reference: http://code.activestate.com/recipes/577594-gps-distance-and-bearing-between-two-gps-points/
    '''
    R = 6371.0
    lat1 = radians(lat1)
    lon1 = radians(lon1)
    lat2 = radians(lat2)
    lon2 = radians(lon2)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = R * c
    return distance * 1000.0

def getBearing(lat1,lon1,lat2,lon2):
    '''
    reference: http://code.activestate.com/recipes/577594-gps-distance-and-bearing-between-two-gps-points/
    '''
    dLon = lon2 - lon1
    y = sin(dLon) * cos(lat2)
    x = cos(lat1) * sin(lat2) \
        - sin(lat1) * cos(lat2) * cos(dLon)
    return atan2(y, x)

def csv_writter(data,path):
    try:
        df = pd.DataFrame(data)
        df.to_csv(path+'output.csv', index=False, header=True)
        print("Data successfully saved.")
    except:
        print("Data could not save.")

