"""
    Class: Vbox3iCanMessage
    Version: 1.0
    --------------------------------------------------

    Class to handle VBOX 3i CAN output message received from COM port (USB-serial)

    --------------------------------------------------

    Details at:
        https://racelogic.support/01VBOX_Automotive/01VBOX_data_loggers/VBOX_3i_Range/VBOX_3i_User_Manual_(All_Variants)/15_-_VB3i_Technical_Properties/VB3i_CAN_Output

        The VBOX 3i has a CAN output which is present on the 5-way connector output.

        Note: Channels highlighted in BLUE are present on Dual Antenna systems only.

        Data format: Motorola
        Baud rate: 500 kbit/s


        VBOX 3i Kalman filter status
        From: https://racelogic.support/01VBOX_Automotive/01VBOX_data_loggers/VBOX_3i_Range/VBOX_3i_Knowledge_base/VBOX_3i_Kalman_Filter_Status

        The table below shows the hex and decimal numbers associated with Kalman filter status.
        The VBOX can have a number of these status flags active at the same time so the resultant number shown in
        VBOX Tools, VBOX Setup or a logged file will be a result of adding together the numbers of the active statuses
        shown below.

        Kalman Filter Status        | Hex       | Decimal
        -------------------------------------------------
            kf_Clear                | 0x0000    | 0
            kf_NEWIMUVersion        | 0x0001    | 1
            kf_TestMode             | 0x0002    | 2
            kf_Use                  | 0x0004    | 4
            kf_Initialised          | 0x0008    | 8
            kf_IMUDataFound         | 0x0010    | 16
            kf_GoodLock             | 0x0020    | 32
            kf_Reset                | 0x0040    | 64
            kf_CoastingTimeout      | 0x0080    | 128
            kf_IMUFound             | 0x0100    | 256
            kf_Initialise           | 0x0200    | 512
            kf_IMUmounting_error    | 0x0800    | 2048


        I.e.: Kalman Filter Status = 317 (13D Hex)

            256: kf_IMUFound
            32: kf_GoodLock
            16: kf_IMUDataFound
            8: kf_Initialised.
            4: kf_Use.
            1: kf_NewIMUVersion.

            256+32+15+8+4+1 = 317
"""


class Vbox3iCanMessage:
    # If Satellites in view < 3 then only Identifier 0x301 transmitted and bytes 2 to 8 are set to 0x00.
    satellites = 0

    # Time since midnight. This is a count of 10 ms intervals since midnight UTC. (5383690 = 53836.90 seconds since
    # midnight or 14 hours, 57 minutes and 16.90 seconds).
    time_since_midnight_utc = 0

    # Position, Latitude in minutes * 100,000 (311924579 = 51 Degrees, 59.24579 Minutes North). This is a true
    # 32 bit signed integer, North being positive.
    position_latitude = 0

    # Position, Longitude in minutes * 100,000 (11882246 = 1 Degrees, 58.82246 Minutes West). This is a true
    # 32 bit signed integer, West being positive.
    position_longitude = 0

    # Velocity, 0.01 kts per bit.
    speed_knots = 0

    # Heading, 0.01° per bit.
    heading = 0

    # Altitude above the WGS 84 ellipsoid, 0.01 m per bit, signed.
    altitude = 0

    # Vertical Velocity, 0.01 m/s per bit, signed.
    vertical_velocity_ms = 0

    # Non used field. Probably reserved
    unused_reserved = 0

    # Status. 8 bit unsigned char.
    #    Bit 0 = VBOX Lite
    #    Bit 1 = Open or Closed CAN Bus (1=open)
    #    Bit 2 = VBOX3
    #    Bit 3 = Logging Status
    status_1 = 0

    # Status is an 8 bit unsigned char.
    #    Bit 0 = Always set
    #    Bit 2 = Brake test started
    #    Bit 3 = Brake trigger active
    #    Bit 4 = DGPS active
    #    Bit 5 = Dual Lock
    status_2 = 0

    # Distance, 0.000078125 m per bit, unsigned. Corrected to trigger point.
    trigger_distance = 0

    # Longitudinal Acceleration, 0.01 g per bit, signed.
    longitudinal_acceleration_g = 0

    # Lateral Acceleration, 0.01 g per bit, signed.
    lateral_acceleration_g = 0

    # Distance traveled since VBOX reset, 0.000078125 m per bit, unsigned.
    distance = 0

    # Time from last brake trigger event. 0.01 seconds per bit.
    trigger_time = 0

    # Velocity at brake trigger point 0.01 kts per bit.
    trigger_speed_knots = 0

    # Velocity Quality, 0.01 km/h per bit.
    speed_quality_kmh = 0

    # True Heading of vehicle, 16 bit signed integer, 0.01° per bit.
    true_heading_da = 0

    # Slip Angle, 16 bit signed integer 0.01° per bit.
    slip_angle_da = 0

    # Pitch Angle, 16 bit signed integer 0.01° per bit.
    pitch_angle_da = 0

    # Lateral Velocity, 16 bit signed integer 0.01 kts per bit.
    lateral_velocity_kmh_da = 0

    # Yaw Rate, 16 bit signed integer 0.01°/s per bit.
    yaw_rate_da = 0

    # Roll Angle, 16 bit signed integer 0.01° per bit.
    roll_angle_da = 0

    # Longitudinal Velocity, 16 bit signed integer 0.01 kts per bit.
    longitudinal_velocity_kph_da = 0

    # Position, Latitude 48 bit signed integer, Latitude * 10,000,000 (minutes). North being positive.
    position_latitude_48bit = 0

    # Pre FW 2.5.0
    # Kalman filter status, 12 bit unsigned integer. See VBOX 3i Kalman Filter Status for details.
    kalman_filter_status = 0

    # Post FW 2.5.0
    # Position Quality, 8 bit unsigned integer.
    position_quality = 0

    # Post FW 2.5.0
    # Solution Type, 8 bit unsigned integer
    #    0 = None
    #    1 = GNSS only
    #    2 = GNSS DGPS
    #    3 = RTK Float
    #    4 = RTK Fixed
    #    5 = Fixed position
    #    6 = IMU Coast
    solution_type = 0

    # Position, Longitude 48 bit signed integer, Longitude *10,000,000 (minutes). East being positive.
    position_longitude_48bit = 0

    # Velocity, 16 bit signed integer 0.01 kts per bit (not delayed when ADAS enabled).
    speed_knots_robot_nav_da = 0

    # Slip Angle Front Left, 16 bit signed integer 0.01° per bit.
    slip_angle_front_left_da = 0

    # Slip Angle Front Right, 16 bit signed integer 0.01° per bit.
    slip_angle_front_right_da = 0

    # Slip Angle Rear Left, 16 bit signed integer 0.01° per bit.
    slip_angle_rear_left_da = 0

    # Slip Angle Rear Right, 16 bit signed integer 0.01° per bit.
    slip_angle_rear_right_da = 0

    # Slip Angle C of G, 16 bit signed integer 0.01° per bit.
    slip_angle_cog_da = 0

    # Robot Navigation Satellites.
    robot_nav_satellites = 0

    # Time since midnight. This is a count of 10 ms intervals since midnight UTC. (5383690 = 53836.90 seconds
    # since midnight or 14 hours, 57 minutes and 16.90 seconds) (not delayed when ADAS enabled).
    robot_nav_time_since_midnight = 0

    # True Heading2 16 bit unsigned integer 0.01° per bit (not delayed when ADAS enabled).
    robot_nav_heading_da = 0

    # Trigger event UTC time - milliseconds since midnight UTC (part 1 of 2 part message).
    trigger_event_utc_part_1 = 0

    # Trigger event UTC time - nanoseconds since midnight UTC (part 2 of 2 part message).
    trigger_event_utc_part_2 = 0

    # Heading derived from the Kalman Filter.
    heading_imu = 0

    # Roll Angle derived from Kalman Filter.
    roll_ang_imu = 0

    # Pitch Angle derived from Kalman Filter.
    pitch_ang_imu = 0

    # Pre FW 2.5.0: Unused
    # Post FW 2.5.0: Kalman filter status, 12 bit unsigned integer. See VBOX 3i Kalman Filter Status for details.
    status_kf = 0

    # Non used field. Probably reserved
    unused_reserved_2 = 0

    # VBOX FW version, 32 bit unsigned. Can be split into
    #    Major (8 bit)
    #    Minor (8 bit)
    #    Build number (16 bit).
    fw_version = 0

    def __init__(self, satellites, time_since_midnight_utc, position_latitude, position_longitude, speed_knots, heading,
                 altitude, vertical_velocity_ms, status_1, status_2, trigger_distance, longitudinal_acceleration_g,
                 lateral_acceleration_g, distance, trigger_time, trigger_speed_knots, speed_quality_kmh,
                 true_heading_da, slip_angle_da, pitch_angle_da, lateral_velocity_kmh_da, yaw_rate_da, roll_angle_da,
                 longitudinal_velocity_kph_da, position_latitude_48bit, kalman_filter_status, position_quality,
                 solution_type, position_longitude_48bit, speed_knots_robot_nav_da, slip_angle_front_left_da,
                 slip_angle_front_right_da, slip_angle_rear_left_da, slip_angle_rear_right_da, slip_angle_cog_da,
                 robot_nav_satellites, robot_nav_time_since_midnight, robot_nav_heading_da, trigger_event_utc_part_1,
                 trigger_event_utc_part_2, heading_imu, roll_ang_imu, pitch_ang_imu, status_kf, fw_version):
        self.satellites = satellites
        self.time_since_midnight_utc = time_since_midnight_utc
        self.position_latitude = position_latitude
        self.position_longitude = position_longitude
        self.speed_knots = speed_knots
        self.heading = heading
        self.altitude = altitude
        self.vertical_velocity_ms = vertical_velocity_ms
        self.status_1 = status_1
        self.status_2 = status_2
        self.trigger_distance = trigger_distance
        self.longitudinal_acceleration_g = longitudinal_acceleration_g
        self.lateral_acceleration_g = lateral_acceleration_g
        self.distance = distance
        self.trigger_time = trigger_time
        self.trigger_speed_knots = trigger_speed_knots
        self.speed_quality_kmh = speed_quality_kmh
        self.true_heading_da = true_heading_da
        self.slip_angle_da = slip_angle_da
        self.pitch_angle_da = pitch_angle_da
        self.lateral_velocity_kmh_da = lateral_velocity_kmh_da
        self.yaw_rate_da = yaw_rate_da
        self.roll_angle_da = roll_angle_da
        self.longitudinal_velocity_kph_da = longitudinal_velocity_kph_da
        self.position_latitude_48bit = position_latitude_48bit
        self.kalman_filter_status = kalman_filter_status
        self.position_quality = position_quality
        self.solution_type = solution_type
        self.position_longitude_48bit = position_longitude_48bit
        self.speed_knots_robot_nav_da = speed_knots_robot_nav_da
        self.slip_angle_front_left_da = slip_angle_front_left_da
        self.slip_angle_front_right_da = slip_angle_front_right_da
        self.slip_angle_rear_left_da = slip_angle_rear_left_da
        self.slip_angle_rear_right_da = slip_angle_rear_right_da
        self.slip_angle_cog_da = slip_angle_cog_da
        self.robot_nav_satellites = robot_nav_satellites
        self.robot_nav_time_since_midnight = robot_nav_time_since_midnight
        self.robot_nav_heading_da = robot_nav_heading_da
        self.trigger_event_utc_part_1 = trigger_event_utc_part_1
        self.trigger_event_utc_part_2 = trigger_event_utc_part_2
        self.heading_imu = heading_imu
        self.roll_ang_imu = roll_ang_imu
        self.pitch_ang_imu = pitch_ang_imu
        self.status_kf = status_kf
        self.fw_version = fw_version

    @staticmethod
    def get_csv_header():
        result = "satellites,time_since_midnight_utc,position_latitude,position_longitude,speed_knots,heading," + \
                 "altitude,vertical_velocity_ms,status_1,status_2,trigger_distance,longitudinal_acceleration_g," + \
                 "lateral_acceleration_g,distance,trigger_time,trigger_speed_knots,speed_quality_kmh," + \
                 "true_heading_da,slip_angle_da,pitch_angle_da,lateral_velocity_kmh_da,yaw_rate_da," + \
                 "roll_angle_da,longitudinal_velocity_kph_da,position_latitude_48bit,kalman_filter_status," + \
                 "position_quality,solution_type,position_longitude_48bit,speed_knots_robot_nav_da," + \
                 "slip_angle_front_left_da,slip_angle_front_right_da,slip_angle_rear_left_da," + \
                 "slip_angle_rear_right_da,slip_angle_cog_da,robot_nav_satellites,robot_nav_time_since_midnight," + \
                 "robot_nav_heading_da,trigger_event_utc_part_1,trigger_event_utc_part_2,heading_imu,roll_ang_imu," + \
                 "pitch_ang_imu,status_kf,fw_version"
        return result

    def get_csv_line(self):
        result = self.satellites + "," + self.time_since_midnight_utc + "," + self.position_latitude + "," + \
                 self.position_longitude + "," + self.speed_knots + "," + self.heading + "," + self.altitude + "," + \
                 self.vertical_velocity_ms + "," + self.status_1 + "," + self.status_2 + "," + self.trigger_distance + "," + \
                 self.longitudinal_acceleration_g + "," + self.lateral_acceleration_g + "," + self.distance + "," + \
                 self.trigger_time + "," + self.trigger_speed_knots + "," + self.speed_quality_kmh + "," + self.true_heading_da + \
                 "," + self.slip_angle_da + "," + self.pitch_angle_da + "," + self.lateral_velocity_kmh_da + "," + self.yaw_rate_da + \
                 "," + self.roll_angle_da + "," + self.longitudinal_velocity_kph_da + "," + self.position_latitude_48bit + "," + \
                 self.kalman_filter_status + "," + self.position_quality + "," + self.solution_type + "," + \
                 self.position_longitude_48bit + "," + self.speed_knots_robot_nav_da + "," + self.slip_angle_front_left_da + "," + \
                 self.slip_angle_front_right_da + "," + self.slip_angle_rear_left_da + "," + self.slip_angle_rear_right_da + "," + \
                 self.slip_angle_cog_da + "," + self.robot_nav_satellites + "," + self.robot_nav_time_since_midnight + "," + \
                 self.robot_nav_heading_da + "," + self.trigger_event_utc_part_1 + "," + self.trigger_event_utc_part_2 + "," + \
                 self.heading_imu + "," + self.roll_ang_imu + "," + self.pitch_ang_imu + "," + self.status_kf + "," + self.fw_version
        return result

    # Status. 8 bit unsigned char.
    #    Bit 0 = VBOX Lite
    #    Bit 1 = Open or Closed CAN Bus (1=open)
    #    Bit 2 = VBOX3
    #    Bit 3 = Logging Status
    def get_descriptive_status_1(self):
        result = ""

        # Check against bitwise operators
        # If the 1st bit is set:
        if self.status_1 & 1 != 0:
            result += "VBOX Lite "

        # if the 2nd bit is set:
        if self.status_1 & 2 != 0:
            result += "CAN Bus open "
        else:
            result += "CAN Bus closed "

        # Check if the 3rd bit is set:
        if self.status_1 & 3 != 0:
            result += "VBOX3 "

        if self.status_1 & 4 != 0:
            result += "logging ON"
        else:
            result += "logging OFF"

    # Status is an 8 bit unsigned char.
    #    Bit 0 = Always set
    #    Bit 2 = Brake test started
    #    Bit 3 = Brake trigger active
    #    Bit 4 = DGPS active
    #    Bit 5 = Dual Lock
    def get_descriptive_status_2(self):
        result = ""

        # Check against bitwise operators
        # If the 1st bit is set:
        if self.status_2 & 1 != 0:
            result += "Always set "

        # if the 2nd bit is set:
        if self.status_2 & 3 != 0:
            result += "Brake test started "
        else:
            result += "Brake test not started "

        # Check if the 3rd bit is set:
        if self.status_2 & 4 != 0:
            result += "Brake trigger active "
        else:
            result += "Brake trigger not active "

        if self.status_2 & 5 != 0:
            result += "DGPS ON "
        else:
            result += "DGPS OFF "

        if self.status_2 & 6 != 0:
            result += "Dual lock ON"
        else:
            result += "Dual lock OFF"

    def get_msg_json(self):
        result = "{\n" + \
                 "\t\"satellites\": " + self.satellites + ",\n" + \
                 "\t\"time_since_midnight_utc\": " + self.time_since_midnight_utc + ",\n" + \
                 "\t\"position_latitude\": " + self.position_latitude + ",\n" + \
                 "\t\"position_longitude\": " + self.position_longitude + ",\n" + \
                 "\t\"speed_knots\": " + self.speed_knots + ",\n" + \
                 "\t\"heading\": " + self.heading + ",\n" + \
                 "\t\"altitude\": " + self.altitude + ",\n" + \
                 "\t\"vertical_velocity_ms\": " + self.vertical_velocity_ms + ",\n" + \
                 "\t\"status_1\": " + self.status_1 + ",\n" + \
                 "\t\"status_2\": " + self.status_2 + ",\n" + \
                 "\t\"trigger_distance\": " + self.trigger_distance + ",\n" + \
                 "\t\"longitudinal_acceleration_g\": " + self.longitudinal_acceleration_g + ",\n" + \
                 "\t\"lateral_acceleration_g\": " + self.lateral_acceleration_g + ",\n" + \
                 "\t\"distance\": " + self.distance + ",\n" + \
                 "\t\"trigger_time\": " + self.trigger_time + ",\n" + \
                 "\t\"trigger_speed_knots\": " + self.trigger_speed_knots + ",\n" + \
                 "\t\"speed_quality_kmh\": " + self.speed_quality_kmh + ",\n" + \
                 "\t\"true_heading_da\": " + self.true_heading_da + ",\n" + \
                 "\t\"slip_angle_da\": " + self.slip_angle_da + ",\n" + \
                 "\t\"pitch_angle_da\": " + self.pitch_angle_da + ",\n" + \
                 "\t\"lateral_velocity_kmh_da\": " + self.lateral_velocity_kmh_da + ",\n" + \
                 "\t\"yaw_rate_da\": " + self.yaw_rate_da + ",\n" + \
                 "\t\"roll_angle_da\": " + self.roll_angle_da + ",\n" + \
                 "\t\"longitudinal_velocity_kph_da\": " + self.longitudinal_velocity_kph_da + ",\n" + \
                 "\t\"position_latitude_48bit\": " + self.position_latitude_48bit + ",\n" + \
                 "\t\"kalman_filter_status\": " + self.kalman_filter_status + ",\n" + \
                 "\t\"position_quality\": " + self.position_quality + ",\n" + \
                 "\t\"solution_type\": " + self.solution_type + ",\n" + \
                 "\t\"position_longitude_48bit\": " + self.position_longitude_48bit + ",\n" + \
                 "\t\"speed_knots_robot_nav_da\": " + self.speed_knots_robot_nav_da + ",\n" + \
                 "\t\"slip_angle_front_left_da\": " + self.slip_angle_front_left_da + ",\n" + \
                 "\t\"slip_angle_front_right_da\": " + self.slip_angle_front_right_da + ",\n" + \
                 "\t\"slip_angle_rear_left_da\": " + self.slip_angle_rear_left_da + ",\n" + \
                 "\t\"slip_angle_rear_right_da\": " + self.slip_angle_rear_right_da + ",\n" + \
                 "\t\"slip_angle_cog_da\": " + self.slip_angle_cog_da + ",\n" + \
                 "\t\"robot_nav_satellites\": " + self.robot_nav_satellites + ",\n" + \
                 "\t\"robot_nav_time_since_midnight\": " + self.robot_nav_time_since_midnight + ",\n" + \
                 "\t\"robot_nav_heading_da\": " + self.robot_nav_heading_da + ",\n" + \
                 "\t\"trigger_event_utc_part_1\": " + self.trigger_event_utc_part_1 + ",\n" + \
                 "\t\"trigger_event_utc_part_2\": " + self.trigger_event_utc_part_2 + ",\n" + \
                 "\t\"heading_imu\": " + self.heading_imu + ",\n" + \
                 "\t\"roll_ang_imu\": " + self.roll_ang_imu + ",\n" + \
                 "\t\"pitch_ang_imu\": " + self.pitch_ang_imu + ",\n" + \
                 "\t\"status_kf\": " + self.status_kf + ",\n" + \
                 "\t\"fw_version\": " + self.fw_version + ",\n" + \
                 "}"
        return result

    def get_msg_json_human_readable(self):
        result = "{\n" + \
                 "\t\"satellites\": " + self.satellites + ",\n" + \
                 "\t\"time_since_midnight_utc\": " + self.time_since_midnight_utc + ",\n" + \
                 "\t\"position_latitude\": " + self.position_latitude + ",\n" + \
                 "\t\"position_longitude\": " + self.position_longitude + ",\n" + \
                 "\t\"speed_knots\": " + self.speed_knots + ",\n" + \
                 "\t\"heading\": " + self.heading + ",\n" + \
                 "\t\"altitude\": " + self.altitude + ",\n" + \
                 "\t\"vertical_velocity_ms\": " + self.vertical_velocity_ms + ",\n" + \
                 "\t\"status_1\": " + self.get_descriptive_status_1() + ",\n" + \
                 "\t\"status_2\": " + self.get_descriptive_status_2() + ",\n" + \
                 "\t\"trigger_distance\": " + self.trigger_distance + ",\n" + \
                 "\t\"longitudinal_acceleration_g\": " + self.longitudinal_acceleration_g + ",\n" + \
                 "\t\"lateral_acceleration_g\": " + self.lateral_acceleration_g + ",\n" + \
                 "\t\"distance\": " + self.distance + ",\n" + \
                 "\t\"trigger_time\": " + self.trigger_time + ",\n" + \
                 "\t\"trigger_speed_knots\": " + self.trigger_speed_knots + ",\n" + \
                 "\t\"speed_quality_kmh\": " + self.speed_quality_kmh + ",\n" + \
                 "\t\"true_heading_da\": " + self.true_heading_da + ",\n" + \
                 "\t\"slip_angle_da\": " + self.slip_angle_da + ",\n" + \
                 "\t\"pitch_angle_da\": " + self.pitch_angle_da + ",\n" + \
                 "\t\"lateral_velocity_kmh_da\": " + self.lateral_velocity_kmh_da + ",\n" + \
                 "\t\"yaw_rate_da\": " + self.yaw_rate_da + ",\n" + \
                 "\t\"roll_angle_da\": " + self.roll_angle_da + ",\n" + \
                 "\t\"longitudinal_velocity_kph_da\": " + self.longitudinal_velocity_kph_da + ",\n" + \
                 "\t\"position_latitude_48bit\": " + self.position_latitude_48bit + ",\n" + \
                 "\t\"kalman_filter_status\": " + self.kalman_filter_status + ",\n" + \
                 "\t\"position_quality\": " + self.position_quality + ",\n" + \
                 "\t\"solution_type\": " + self.solution_type + ",\n" + \
                 "\t\"position_longitude_48bit\": " + self.position_longitude_48bit + ",\n" + \
                 "\t\"speed_knots_robot_nav_da\": " + self.speed_knots_robot_nav_da + ",\n" + \
                 "\t\"slip_angle_front_left_da\": " + self.slip_angle_front_left_da + ",\n" + \
                 "\t\"slip_angle_front_right_da\": " + self.slip_angle_front_right_da + ",\n" + \
                 "\t\"slip_angle_rear_left_da\": " + self.slip_angle_rear_left_da + ",\n" + \
                 "\t\"slip_angle_rear_right_da\": " + self.slip_angle_rear_right_da + ",\n" + \
                 "\t\"slip_angle_cog_da\": " + self.slip_angle_cog_da + ",\n" + \
                 "\t\"robot_nav_satellites\": " + self.robot_nav_satellites + ",\n" + \
                 "\t\"robot_nav_time_since_midnight\": " + self.robot_nav_time_since_midnight + ",\n" + \
                 "\t\"robot_nav_heading_da\": " + self.robot_nav_heading_da + ",\n" + \
                 "\t\"trigger_event_utc_part_1\": " + self.trigger_event_utc_part_1 + ",\n" + \
                 "\t\"trigger_event_utc_part_2\": " + self.trigger_event_utc_part_2 + ",\n" + \
                 "\t\"heading_imu\": " + self.heading_imu + ",\n" + \
                 "\t\"roll_ang_imu\": " + self.roll_ang_imu + ",\n" + \
                 "\t\"pitch_ang_imu\": " + self.pitch_ang_imu + ",\n" + \
                 "\t\"status_kf\": " + self.status_kf + ",\n" + \
                 "\t\"fw_version\": " + self.fw_version + ",\n" + \
                 "}"
        return result
