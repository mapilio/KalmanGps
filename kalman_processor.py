from addict import Dict
from data import get_data_csv
import numpy as np
from madgwickahrs import MadgwickAHRS
from statistics import mean
import math
from utils import *
import gmplot


class Kalman():

    def __init__(self, csv_path=None):
        self.samples = None
        self.csv_path = csv_path

    def __call__(self, **kwargs):

        self.get_data(**kwargs)
        self.check_requires()
        self.samples = len(self.data['accx'])
        print("Interpolating...")

        # !!!Attention!!!  TODO
        # using numpy linear interpolation
        self.gpsTimeStampInterp = np.linspace(self.data['timestamp_ns'][0], self.data['timestamp_ns'][-1],
                                              self.samples)
        self.gpsLatitudeInterpolated = np.interp(self.gpsTimeStampInterp, self.data['timestamp_ns'],
                                                 self.data['lat'])
        self.gpsLongitudeInterpolated = np.interp(self.gpsTimeStampInterp, self.data['timestamp_ns'],
                                                  self.data['lon'])
        # gpsHeadingInterpolated = np.interp(gpsTimeStampInterp, self.data['timestamp_ns'], self.data['heading'])

        # taking GPS time stamps as they GPS is absolute data
        accTimeStampInterp = np.linspace(self.gpsTimeStampInterp[0], self.gpsTimeStampInterp[- 1], self.samples)
        accXInterpolated = np.interp(accTimeStampInterp, self.data['timestamp_ns'], self.data['accx'])
        accYInterpolated = np.interp(accTimeStampInterp, self.data['timestamp_ns'], self.data['accy'])
        accZInterpolated = np.interp(accTimeStampInterp, self.data['timestamp_ns'], self.data['accz'])

        gyroTimeStampInterp = np.linspace(self.gpsTimeStampInterp[0], self.gpsTimeStampInterp[- 1], self.samples)
        gyroXInterpolated = np.interp(gyroTimeStampInterp, self.data['timestamp_ns'], self.data['gyrx'])
        gyroYInterpolated = np.interp(gyroTimeStampInterp, self.data['timestamp_ns'], self.data['gyry'])
        gyroZInterpolated = np.interp(gyroTimeStampInterp, self.data['timestamp_ns'], self.data['gyrz'])

        magTimeStampInterp = np.linspace(self.gpsTimeStampInterp[0], self.gpsTimeStampInterp[- 1], self.samples)
        magXInterpolated = np.interp(magTimeStampInterp, self.data['timestamp_ns'], self.data['magx'])
        magYInterpolated = np.interp(magTimeStampInterp, self.data['timestamp_ns'], self.data['magy'])
        magZInterpolated = np.interp(magTimeStampInterp, self.data['timestamp_ns'], self.data['magz'])

        self.dt = [(self.gpsTimeStampInterp[i + 1] - self.gpsTimeStampInterp[i]) * 0.000000001 for i in
                   range(len(self.gpsTimeStampInterp) - 1)]
        self.dt.insert(0, 0)

        self.accXAbsolute = []
        self.accYAbsolute = []

        print("Calculating absolute acc values...")
        heading = MadgwickAHRS(sampleperiod=mean(self.dt))
        for i in range(self.samples):
            gyroscope = [gyroZInterpolated[i], gyroYInterpolated[i], gyroXInterpolated[i]]
            accelerometer = [accZInterpolated[i], accYInterpolated[i], accXInterpolated[i]]
            magnetometer = [magZInterpolated[i], magYInterpolated[i], magXInterpolated[i]]
            heading.update(gyroscope, accelerometer, magnetometer)
            ahrs = heading.quaternion.to_euler_angles()
            roll = ahrs[0]
            pitch = ahrs[1]
            yaw = ahrs[2] + (3.0 * (math.pi / 180.0))  # adding magenetic declination
            ACC = np.array([[self.data['accz'][i]], [self.data['accy'][i]], [self.data['accx'][i]]])
            ACCABS = np.linalg.inv(rotaitonMatrix(yaw, pitch, roll)).dot(ACC)
            self.accXAbsolute.append(-1 * ACCABS[0, 0])
            self.accYAbsolute.append(-1 * ACCABS[1, 0])

        self.set_kalman_parameters()
        self.perform_kalman()
        self.extract_map()
        return {'lat': self.Yfilter,
                'lon': self.Xfilter}

    def perform_kalman(self):
        self.Xfilter = []
        self.Yfilter = []
        print("Running kalman filter...")
        for i in range(len(self.dt)):
            self.X_hat_t, self.P_hat_t = prediction(self.X_hat_t, self.P_t, self.F_t, self.B_t, self.U_t,
                                                    self.Q_t)  # STATE PREDICTION (IMU)
            Z_t = np.array([[self.gpsLongitudeInterpolated[i]], [self.gpsLatitudeInterpolated[i]]])
            Z_t = Z_t.reshape(Z_t.shape[0], -1)
            X_t, self.P_t = update(self.X_hat_t, self.P_hat_t, Z_t, self.R_t, self.H_t)  # MEASUREMENT UPDATE (GPS)
            self.X_hat_t = X_t
            self.P_hat_t = self.P_t
            self.F_t = np.array([[1, 0, self.dt[i], 0], [0, 1, 0, self.dt[i]], [0, 0, 1, 0], [0, 0, 0, 1]])
            self.B_t = np.array(
                [[0.5 * self.dt[i] ** 2, 0, 0, 0], [0, 0.5 * self.dt[i] ** 2, 0, 0], [0, 0, self.dt[i], 0],
                 [0, 0, 0, self.dt[i]]])
            self.U_t = np.array(
                [[self.accXAbsolute[i]], [self.accYAbsolute[i]], [self.accXAbsolute[i]], [self.accYAbsolute[i]]])
            self.Xfilter.append(X_t[0, 0])
            self.Yfilter.append(X_t[1, 0])

    def set_kalman_parameters(self):
        # Transition matrix
        self.F_t = np.array([[1, 0, self.dt[0], 0], [0, 1, 0, self.dt[0]], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Initial State cov
        self.P_t = np.identity(4) * 400
        # Process cov
        self.Q_t = np.array([[1000, 0, 100, 0], [0, 1000, 0, 100], [100, 0, 1000, 0], [0, 100, 0, 1000]]) * 0.65
        # Control matrix
        self.B_t = np.array([[0.5 * self.dt[0] ** 2, 0, 0, 0], [0, 0.5 * self.dt[0] ** 2, 0, 0], [0, 0, self.dt[0], 0],
                             [0, 0, 0, self.dt[0]]])
        # Control vector
        self.U_t = np.array(
            [[self.accXAbsolute[0]], [self.accYAbsolute[0]], [self.accXAbsolute[0]], [self.accYAbsolute[0]]])
        # Measurment Matrix
        self.H_t = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        # Measurment cov
        self.R_t = np.identity(2)
        # Initial State
        self.X_hat_t = np.array([[self.gpsLongitudeInterpolated[0]], [self.gpsLatitudeInterpolated[0]], [0], [0]])

    def get_data(self, **kwargs):
        try:
            if self.csv_path is not None:
                self.data = get_data_csv(csv_file=self.csv_path)
            else:
                self.data = kwargs
        except:
            print("Data cant obtained")

    def extract_map(self):
        # reducing data points to 100 for plotting on google maps
        # dtReduced = np.linspace(self.data['timestamp_ns'][0], self.data['timestamp_ns'][- 1], 100)
        # XfilterInterp = np.interp(dtReduced, self.gpsTimeStampInterp, self.Xfilter)
        # YfilterInterp = np.interp(dtReduced, self.gpsTimeStampInterp, self.Yfilter)

        print("Plotting on google maps...")
        gmap1 = gmplot.GoogleMapPlotter(self.data['lat'][0], self.data['lon'][0], 20,
                                        map_type='hybrid')
        gmap1.apikey = "AIzaSyDvvwPIEA8T9IUxPKaRZ6gp2f6xRtBYICU"
        gmap1.scatter(self.data['lat'], self.data['lon'], color='r', size=0.3, marker=False)
        gmap1.scatter(self.Yfilter, self.Xfilter, color='g', size=0.4, marker=False)
        gmap1.draw("results_clss.html")

        print("Done...")




if __name__ == '__main__':
    csv_path = 'sensor_data/20-10-23_1.csv'
    klmn = Kalman(csv_path=csv_path)
    klmn()
