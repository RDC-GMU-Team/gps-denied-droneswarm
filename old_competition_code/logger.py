import zulu

class Logger:
    filename = "flight.log"
    def __init__(self):
        open(self.filename, 'w').close()  # clear file
    
    def log(self, pos, aruco_id):
        comp = 'RTXDC_2024'
        school_name = 'GEORGE_MASON_UNIVERSITY'
        date_time_stamp = zulu.now()
        logfile = open(self.filename, 'a')
        logfile.write(f"{comp} {school_name}_UAV_WaterBlast!_{aruco_id}_{date_time_stamp}_{pos['lat']}_{pos['lon']}")
        logfile.close()