import pandas as pd


def get_data_csv(csv_file: str = None):
    if csv_file is None:
        print("csv file could not found.")
    print("Reading files...")
    # Read the CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    # Convert the "current_time" column to datetime objects
    df['current_time'] = pd.to_datetime(df['current_time'])

    # Convert datetime objects to timestamps with nanoseconds
    df['timestamp_ns'] = df['current_time'].astype(int)
    df.set_index('timestamp_ns')

    # Get rows between 'start_row' and 'end_row'
    # df = df.iloc[10:-10]

    # Convert the DataFrame into a dictionary
    data_dict = df.to_dict(orient='list')
    data_dict['timestamp_ns'] = [int(str(row)[:-3]) for row in data_dict['timestamp_ns']]
    return data_dict


def modify_time(data):
    df = pd.DataFrame(data)

    # Convert the "current_time" column to datetime objects
    df['current_time'] = pd.to_datetime(df['current_time'])

    # Convert datetime objects to timestamps with nanoseconds
    df['timestamp_ns'] = df['current_time'].astype(int)
    df.set_index('timestamp_ns')

    # Convert the DataFrame into a dictionary
    data_dict = df.to_dict(orient='list')
    data_dict['timestamp_ns'] = [int(str(row)[:-3]) for row in data_dict['timestamp_ns']]
    return data_dict


def get_data_txt(txt_file):
    # Defining file locations
    gpsDatafileName = "sensor_data/gpsValues.txt"
    accDatafileName = "sensor_data/accelerometerValues.txt"
    gyroDatafileName = "sensor_data/gyroscopeValues.txt"
    magDatafileName = "sensor_data/magneticValues.txt"

    # defining file handles
    gpsValuesFileHandle = open(gpsDatafileName, 'r')
    accValuesFileHandle = open(accDatafileName, 'r')
    gyroValuesFileHandle = open(gyroDatafileName, 'r')
    magValuesFileHandle = open(magDatafileName, 'r')

    GPSDATA, gpsTimeStamp, gpsLatitude, gpsLongitude, gpsAltitude, gpsBearing, gpsSpeed = ([] for i in range(7))
    ACCDATA, accTimeStamp, accX, accY, accZ = ([] for i in range(5))
    GYRODATA, gyroTimeStamp, gyroX, gyroY, gyroZ = ([] for i in range(5))
    MAGDATA, magTimeStamp, magX, magY, magZ = ([] for i in range(5))

    for gpsData in gpsValuesFileHandle:
        gpsData = gpsData.rstrip()
        GPSDATA.append(gpsData.split(','))
    for i in range(6, len(GPSDATA)):  # ignoring erronious data points
        gpsTimeStamp.append(float(GPSDATA[i][0]))
        gpsLatitude.append(float(GPSDATA[i][1]))
        gpsLongitude.append(float(GPSDATA[i][2]))
        gpsBearing.append(float(GPSDATA[i][4]))
    for accData in accValuesFileHandle:
        accData = accData.rstrip()
        ACCDATA.append(accData.split(','))
    for i in range(1, len(ACCDATA)):
        accTimeStamp.append(float(ACCDATA[i][0]))
        accX.append(float(ACCDATA[i][1]))
        accY.append(float(ACCDATA[i][2]))
        accZ.append(float(ACCDATA[i][3]))
    for gyroData in gyroValuesFileHandle:
        gyroData = gyroData.rstrip()
        GYRODATA.append(gyroData.split(','))
    for i in range(1, len(GYRODATA)):
        gyroTimeStamp.append(float(GYRODATA[i][0]))
        gyroX.append(float(GYRODATA[i][1]))
        gyroY.append(float(GYRODATA[i][2]))
        gyroZ.append(float(GYRODATA[i][3]))
    for magData in magValuesFileHandle:
        magData = magData.rstrip()
        MAGDATA.append(magData.split(','))
    for i in range(1, len(MAGDATA)):
        magTimeStamp.append(float(MAGDATA[i][0]))
        magX.append(float(MAGDATA[i][1]))
        magY.append(float(MAGDATA[i][2]))
        magZ.append(float(MAGDATA[i][3]))

    return gpsTimeStamp, gpsLatitude, gpsLongitude, gpsAltitude, gpsBearing, \
        accTimeStamp, accX, accY, accZ, \
        gyroTimeStamp, gyroX, gyroY, gyroZ, \
        magTimeStamp, magX, magY, magZ
