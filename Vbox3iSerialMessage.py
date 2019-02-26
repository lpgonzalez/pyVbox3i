"""
    Class: Vbox3iSerialMessage
    Version: 1.0
    --------------------------------------------------

    Class to handle VBOX 3i Serial (RS232) message received from COM port (USB-serial)

    --------------------------------------------------

    Details at:
        https://racelogic.support/01VBOX_Automotive/01VBOX_data_loggers/VBOX_3i_Range/VBOX_3i_Knowledge_base/VBOX_3i_Serial_(RS232)_Protocol

        Message format:
            $VBOX3i,nnnn0000,stttaaaaoooovvhheeezzxxyyffffjjjj1111222233334444lpddddddqqrrii77778888kkmmmgggg559966cc

            The $VBOX3i and commas are in ASCII, the rest is in binary.

        Baud rate: 115200 Baud, no parity, 8 data bits, 1 stop bit

        The channels contained in the message depend on the first 4 bytes (nnnn - as 8 characters of hexadecimal).


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


        CRC calculation
        ---------------

        CRC Calculation example :

            s[n] is a string containing the message
            Polynomial:= 4129
            CRC:=0;
            for Loop:=1 to Length(s) do
            begin
                Temp:=s[Loop];
                CRC:= CRC xor (integer(Temp) * 256);
                CRC:= CRC mod 65536;
                for i:=7 downto 0 do
                begin
                    if ( (CRC and 32768)=32768) then
                    begin
                        CRC:= CRC *2 ;
                        CRC:= CRC xor Polynomial;
                    end
                    else
                    begin
                        CRC:= CRC *2 ;
                    end;
                    CRC:=CRC mod 65536;
                end;
            end;
            result:=CRC;
"""


class Vbox3iSerialMessage:
    # nnnn, bit mask. Reserved to indicate channel presence.
    #   part: nnnn
    #   format: -
    #   bytes: 4
    #   nnnn,bit mask: -
    msg_configuration = 0

    # Reserved
    #   part: 0000
    #   format: -
    #   bytes: 4
    #   nnnn,bit mask: -
    reserved = 0

    # Satellites (number of satellites)
    #   part: s
    #   format: Integer
    #   bytes: 1
    #   nnnn,bit mask: 0x00000001
    satellites = 0

    # Time (number of 10 ms ticks since midnight UTC)
    #   part: ttt
    #   format: Integer
    #   bytes: 3
    #   nnnn,bit mask: 0x00000002
    time_since_midnight_utc = 0

    # Latitude ( MMMM.MMMMM * 100,000
    #            Signed Integer of Decimal minutes *100,000.
    #            Positive = North, Negative = South )
    #   part: aaaa
    #   format: Signed
    #   bytes: 4
    #   nnnn,bit mask: 0x00000004
    latitude = 0

    # Longitude ( MMMMM.MMMMM * 100,000
    #             Signed Integer of Decimal minutes *100,000.
    #             Positive = West, Negative = East )
    #   part: oooo
    #   format: Signed
    #   bytes: 4
    #   nnnn,bit mask: 0x00000008
    longitude = 0

    # Velocity (in Knots * 100)
    #   part: vv
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x00000010
    speed_knots = 0

    # Heading (degrees from true north * 100)
    #   part: hh
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x00000020
    heading = 0

    # Height (altitude. In metres WGS84 * 100
    #         True signed 24 bit number )
    #   part: eee
    #   format: Integer
    #   bytes: 3
    #   nnnn,bit mask: 0x00000040
    height = 0

    # Vertical Velocity (vertical velocity in m/s * 100 )
    #   part: zz
    #   format: Signed
    #   bytes: 2
    #   nnnn,bit mask: 0x00000080
    vertical_velocity_ms = 0

    # Long acc (GPS) (longitudinal acceleration in g * 100 )
    #   part: xx
    #   format: Signed
    #   bytes: 2
    #   nnnn,bit mask: 0x00000100
    long_acc_gps = 0

    # Lat acc (GPS) (Lateral acceleration in g * 100 )
    #   part: yy
    #   format: Signed
    #   bytes: 2
    #   nnnn,bit mask: 0x00000200
    lat_acc_gps = 0

    # Brake Distance (m *12800)
    #   part: ffff
    #   format: Integer
    #   bytes: 4
    #   nnnn,bit mask: 0x00000400
    brake_distance = 0

    # Distance (m *12800)
    #   part: jjjj
    #   format: Integer
    #   bytes: 4
    #   nnnn,bit mask: 0x00000800
    distance = 0

    # Internal analogue ch1
    #   part: 1111
    #   format: Float
    #   bytes: 4
    #   nnnn,bit mask: 0x00001000
    int_analog_ch1 = 0

    # Internal analogue ch2
    #   part: 2222
    #   format: Float
    #   bytes: 4
    #   nnnn,bit mask: 0x00002000
    int_analog_ch2 = 0

    # Internal analogue ch3
    #   part: 3333
    #   format: Float
    #   bytes: 4
    #   nnnn,bit mask: 0x00004000
    int_analog_ch3 = 0

    # Internal analogue ch4
    #   part: 4444
    #   format: Float
    #   bytes: 4
    #   nnnn,bit mask: 0x00008000
    int_analog_ch4 = 0

    # GLONASS satellites (number of GLONASS satellites)
    #   part: l
    #   format: Integer
    #   bytes: 1
    #   nnnn,bit mask: 0x00010000
    glonass_satellites = 0

    # GPS satellites (number of GPS satellites)
    #   part: p
    #   format: Integer
    #   bytes: 1
    #   nnnn,bit mask: 0x00020000
    gps_satellites = 0

    # Reserved (1)
    #   part: dd
    #   format: -
    #   bytes: 2
    #   nnnn,bit mask: 0x00040000
    reserved_1 = 0

    # Reserved (2)
    #   part: dd
    #   format: -
    #   bytes: 2
    #   nnnn,bit mask: 0x00080000
    reserved_2 = 0

    # Reserved (3)
    #   part: dd
    #   format: -
    #   bytes: 2
    #   nnnn,bit mask: 0x00100000
    reserved_3 = 0

    # VBOX Serial Number
    #   part: qq
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x00200000
    vbox_serial_number = 0

    # Kalman Filter Status (see VBOX 3i Kalman Filter Status for details)
    #   part: rr
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x00400000
    kalman_filter_status = 0

    # Solution Type (see VBOX 3i Kalman Filter Status for details)
    #   part: ii
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x00800000
    #
    # Possible codes (from CAN message)
    #    0 = None
    #    1 = GNSS only
    #    2 = GNSS DGPS
    #    3 = RTK Float
    #    4 = RTK Fixed
    #    5 = Fixed position
    #    6 = IMU Coast
    solution_type = 0

    # Velocity Quality ( km/h * 100 )
    #   part: 7777
    #   format: Integer
    #   bytes: 4
    #   nnnn,bit mask: 0x01000000
    velocity_quality = 0

    # Internal Temperature
    #   part: 8888
    #   format: Signed
    #   bytes: 4
    #   nnnn,bit mask: 0x02000000
    internal_temperature = 0

    # CF Buffer size
    #   part: kk
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x04000000
    cf_buffer_size = 0

    # RAM Address (free space on CF *980991 = Full, 0 = Empty)
    #   part: mmm
    #   format: Integer
    #   bytes: 3
    #   nnnn,bit mask: 0x08000000
    ram_address = 0

    # Event time 1
    #   part: gggg
    #   format: Float
    #   bytes: 4
    #   nnnn,bit mask: 0x10000000
    event_time_1 = 0

    # Event time 2
    #   part: 55
    #   format: Float
    #   bytes: 2
    #   nnnn,bit mask: 0x20000000
    event_time_2 = 0

    # Battery 1 Voltage
    #   part: 99
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x40000000
    batt_1_voltage = 0

    # Battery 2 Voltage
    #   part: 66
    #   format: Integer
    #   bytes: 2
    #   nnnn,bit mask: 0x80000000
    batt_2_voltage = 0

    # Checksum (CRC of message, See CRC calculation)
    #   part: cc
    #   format: -
    #   bytes: 2
    #   nnnn,bit mask: -
    checksum = 0

    def __init__(self, msg_configuration, satellites, time_since_midnight_utc, latitude, longitude, speed_knots,
                 heading, height, vertical_velocity_ms, long_acc_gps, lat_acc_gps, brake_distance, distance,
                 int_analog_ch1, int_analog_ch2, int_analog_ch3, int_analog_ch4, glonass_satellites, gps_satellites,
                 vbox_serial_number, kalman_filter_status, solution_type, velocity_quality, internal_temperature,
                 cf_buffer_size, ram_address, event_time_1, event_time_2, batt_1_voltage, batt_2_voltage, checksum):
        self.msg_configuration = msg_configuration
        self.satellites = satellites
        self.time_since_midnight_utc = time_since_midnight_utc
        self.latitude = latitude
        self.longitude = longitude
        self.speed_knots = speed_knots
        self.heading = heading
        self.height = height
        self.vertical_velocity_ms = vertical_velocity_ms
        self.long_acc_gps = long_acc_gps
        self.lat_acc_gps = lat_acc_gps
        self.brake_distance = brake_distance
        self.distance = distance
        self.int_analog_ch1 = int_analog_ch1
        self.int_analog_ch2 = int_analog_ch2
        self.int_analog_ch3 = int_analog_ch3
        self.int_analog_ch4 = int_analog_ch4
        self.glonass_satellites = glonass_satellites
        self.gps_satellites = gps_satellites
        self.vbox_serial_number = vbox_serial_number
        self.kalman_filter_status = kalman_filter_status
        self.solution_type = solution_type
        self.velocity_quality = velocity_quality
        self.internal_temperature = internal_temperature
        self.cf_buffer_size = cf_buffer_size
        self.ram_address = ram_address
        self.event_time_1 = event_time_1
        self.event_time_2 = event_time_2
        self.batt_1_voltage = batt_1_voltage
        self.batt_2_voltage = batt_2_voltage
        self.checksum = checksum

    def __init__(self, msg):
        """
        TO-DO: trim message to init fields
        :param msg: whole RAW message

        self.msg_configuration = msg_configuration
        self.satellites = satellites
        self.time_since_midnight_utc = time_since_midnight_utc
        self.latitude = latitude
        self.longitude = longitude
        self.speed_knots = speed_knots
        self.heading = heading
        self.height = height
        self.vertical_velocity_ms = vertical_velocity_ms
        self.long_acc_gps = long_acc_gps
        self.lat_acc_gps = lat_acc_gps
        self.brake_distance = brake_distance
        self.distance = distance
        self.int_analog_ch1 = int_analog_ch1
        self.int_analog_ch2 = int_analog_ch2
        self.int_analog_ch3 = int_analog_ch3
        self.int_analog_ch4 = int_analog_ch4
        self.glonass_satellites = glonass_satellites
        self.gps_satellites = gps_satellites
        self.vbox_serial_number = vbox_serial_number
        self.kalman_filter_status = kalman_filter_status
        self.solution_type = solution_type
        self.velocity_quality = velocity_quality
        self.internal_temperature = internal_temperature
        self.cf_buffer_size = cf_buffer_size
        self.ram_address = ram_address
        self.event_time_1 = event_time_1
        self.event_time_2 = event_time_2
        self.batt_1_voltage = batt_1_voltage
        self.batt_2_voltage = batt_2_voltage
        self.checksum = checksum
        """

    @staticmethod
    def get_csv_header():
        result = "msg_configuration,satellites,time_since_midnight_utc,latitude,longitude,speed_knots,heading" + \
                 "height,vertical_velocity_ms,long_acc_gps,lat_acc_gps,brake_distance,distance,int_analog_ch1," + \
                 "int_analog_ch2,int_analog_ch3,int_analog_ch4,glonass_satellites,gps_satellites," + \
                 "vbox_serial_number,kalman_filter_status,solution_type,velocity_quality,internal_temperature," + \
                 "cf_buffer_size,ram_address,event_time_1,event_time_2,batt_1_voltage,batt_2_voltage,checksum"
        return result

    # TO-DO
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

    # Possible codes (from CAN message)
    #    0 = None
    #    1 = GNSS only
    #    2 = GNSS DGPS
    #    3 = RTK Float
    #    4 = RTK Fixed
    #    5 = Fixed position
    #    6 = IMU Coast
    def get_descriptive_solution_type(self):
        result = ""

        # Check against bitwise operators
        # If the 1st bit is set:
        if self.solution_type & 1 != 0:
            result += "GNSS only "

        # if the 2nd bit is set:
        if self.solution_type & 2 != 0:
            result += "GNSS DGPS "

        # if the 2nd bit is set:
        if self.solution_type & 3 != 0:
            result += "RTK Float "

        # Check if the 3rd bit is set:
        if self.solution_type & 4 != 0:
            result += "RTK Fixed "

        if self.solution_type & 5 != 0:
            result += "Fixed position "

        if self.solution_type & 6 != 0:
            result += "IMU Coast"

    # TO-DO
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

    # TO-DO
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
