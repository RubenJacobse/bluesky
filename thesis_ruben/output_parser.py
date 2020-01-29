"""
Process the log output files written by the area restriction plugin
and
"""

# Python imports
import os
import csv

# BlueSky imports
from bluesky import settings
settings.set_variable_defaults(log_start=1800,
                               log_end=9000)

# Module level constants
PZ_RADIUS = 5.0 * 1852  # Protected zone radius in meters

# Discard all data outside logging interval
T_LOG_INTERVAL_START = settings.log_start # [s]
T_LOG_INTERVAL_END = settings.log_end # [s]


class LogListParser:
    """
    Parse and process the contents of all log files that are
    in 'input_list' and write the processed data to 'output_file'.
    """

    def __init__(self, input_list, output_file):
        self.input_list = input_list
        self.output_file = output_file

        # Set the default output header which may be overriden in
        # subclasses by using the set_header() method.
        self.header = ""
        self.set_header()

        # Create a new output file and write its header
        with open(self.output_file, "w") as output:
            output.write("#" + self.header + "\n")

        # Parse the contents of each file in the input list
        for filename in self.input_list:
            self.parse_file(filename)

    def parse_file(self, logfile):
        """
        Read the data in logfile and store the metadata in a new file.
        """

        # Read the contents of the file (NOTE: we parse the entire file
        # and store it in a list, this might fail for very large files)
        with open(logfile) as logfile_obj:
            if not os.path.getsize(logfile):
                raise ValueError(f"File \"{logfile}\" is empty!")

            log_reader = csv.reader(logfile_obj, delimiter=",")
            log_data = [row for row in list(log_reader)
                        if not row[0].startswith("#")]

            # Process the list containing the current file's contents and write
            # the summarized data to the output file.
            # Expected logfile name format:
            # "...\LOGNAME_YYYYMMDD-hhmmss_Lxx_Wxx_Axx_RESO-xx_T-xx_SCENxxx.log"
            namesplit = logfile.split("_")
            geometry = namesplit[-6] + "_" + namesplit[-5] + "_" + namesplit[-4]
            reso_method = namesplit[-3][5:]
            traffic_level = namesplit[-2][2:]
            scenario = namesplit[-1][4:7]

            self.summarize_stats(logfile,
                                 log_data,
                                 geometry,
                                 reso_method,
                                 scenario,
                                 traffic_level)

    def write_to_output_file(self, line):
        """
        Write a line containing the relevant parameters of the current
        logfile to the output file.
        """

        # Output file already exists, thus we append to it
        with open(self.output_file, "a") as output:
            output.write(line + "\n")

    def write_lines_to_output_file(self, linelist):
        """
        Write a list of lines containing the relevant parameters of the
        current logfile to the output file.
        """

        # Output file already exists, thus we append to it
        with open(self.output_file, "a") as output:
            for line in linelist:
                output.write(line + "\n")

    # Virtual methods
    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        raise NotImplementedError

    def set_header(self):
        raise NotImplementedError


class AREALogSummaryParser(LogListParser):
    """
    Parse and process the contents of all AREA_LOG_... files that are
    in 'input_list'.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and write the
        summary to logfile.
        """

        num_conflicts = 0
        num_intrusions = 0
        conflict_dict = {}
        intrusion_dict = {}

        for row in log_data:
            [simt, ac_id, area_id, t_int, ac_lat, ac_lon] = row
            simt = int(float(simt))
            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            t_int = float(t_int)
            ac_lat = float(ac_lat)
            ac_lon = float(ac_lon)

            confpair = f"{ac_id}-{area_id}"

            # Check if conflict count needs to be incremented
            if confpair not in conflict_dict.keys():
                conflict_dict[confpair] = []
            if conflict_dict[confpair]:
                if not conflict_dict[confpair][-1] == simt-1:
                    num_conflicts += 1
            else:
                num_conflicts += 1
            conflict_dict[confpair].append(simt)

            # Intrusions are flagged using t_int = 0
            if t_int != 0:
                continue

            # Check if intrusion count needs to be incremented
            if confpair not in intrusion_dict.keys():
                intrusion_dict[confpair] = []
            if intrusion_dict[confpair]:
                if not intrusion_dict[confpair][-1] == simt-1:
                    num_intrusions += 1
            else:
                num_intrusions += 1
            intrusion_dict[confpair].append(simt)

        outputline = (f"{geometry},{reso_method},{traffic_level},"
                      + f"{scenario},{num_conflicts},{num_intrusions}")
        self.write_to_output_file(outputline)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,"
                       + "scenario,num conflicts,num intrusions")


class AREALogLocationParser(LogListParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and write all coordinates at which aircraft are in
    conflict to 'output_file'.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        outputlines = []

        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, _, _, t_int, ac_lat, ac_lon] = row
            simt = int(float(simt))
            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            t_int = float(t_int)
            ac_lat = float(ac_lat)
            ac_lon = float(ac_lon)

            is_int = t_int == 0

            # Write lines for ac1 and ac2 at once
            outputlines.append(f"{geometry},{reso_method},{traffic_level},"
                               + f"{scenario},{ac_lat:0.6f},{ac_lon:0.6f},"
                               + f"{is_int}")

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac lat [deg],ac lon[deg],is intrusion [-]")


class ASASLogSummaryParser(LogListParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and summarize the stats for each scenario.
    """

    def __init__(self, input_list, output_file):
        """
        Extend LogListParser functionality to allow calculation of the
        Domino Effect Parameter (DEP) after the summary file has been created.
        This is necessary because each scenario needs to be compared to the
        case in which aircraft-aircraft resolution methods are off.
        """

        self.summary_list = []
        super().__init__(input_list, output_file)
        self.calculate_domino_effect_parameter()

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and write the
        summary to 'self.summary_list'.
        """

        num_los = 0
        num_conf = 0

        for row in log_data:
            [simt, _, _, is_los, _, _, _, _] = row
            simt = int(float(simt))
            is_los = int(is_los)

            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            if is_los:
                num_los += 1
            else:
                num_conf += 1

        int_prev_rate = (num_conf - num_los) / num_conf

        current_stats = {"geometry": geometry,
                         "reso_method": reso_method,
                         "traffic_level": traffic_level,
                         "scenario": scenario,
                         "num_conf": num_conf,
                         "num_los": num_los,
                         "int_prev_rate": int_prev_rate}

        self.summary_list.append(current_stats)

    def calculate_domino_effect_parameter(self):
        """
        For each run calculate the domino effect parameter relative to the
        same scenario baseline with resolution method "OFF".
        """

        for scen in self.summary_list:
            if scen["reso_method"] == "OFF":
                scen["dep"] = 0
            else:
                basescen = self.get_baseline_stats(scen["geometry"],
                                                   scen["traffic_level"],
                                                   scen["scenario"])
                scen["dep"] = (scen["num_conf"] / basescen["num_conf"]) - 1

            outputline = (f'{scen["geometry"]},{scen["reso_method"]},'
                          + f'{scen["traffic_level"]},{scen["scenario"]},'
                          + f'{scen["num_conf"]},{scen["num_los"]},'
                          + f'{scen["int_prev_rate"]:.3f},{scen["dep"]:.3f}')
            self.write_to_output_file(outputline)

    def get_baseline_stats(self, geometry, traffic_level, scenario):
        """
        Returns the baseline scenario summary (reso method "OFF") for
        the given geometry, traffic_level, and scenario combination.

        Raises a ValueError if no baseline scenario summary is found.
        """

        for scen in self.summary_list:
            if (scen["geometry"] == geometry
                    and scen["reso_method"] == "OFF"
                    and scen["traffic_level"] == traffic_level
                    and scen["scenario"] == scenario):
                return scen

        raise ValueError(f"Baseline scenario stats not found.")

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "num conflicts [-],num LoS [-],IPR [-],DEP [-]")


class ASASLogOccurrenceParser(LogListParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and summarize the stats for each individual conflict.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        outputlines = []

        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, ac1_id, ac2_id, is_los, los_severity, duration, delta_v,
             delta_trk] = row
            simt = int(float(simt))

            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            confpair = f"{ac1_id}-{ac2_id}"
            is_los = str(bool(int(is_los)))
            los_severity = float(los_severity)
            duration = int(duration)
            start = simt - duration
            end = simt

            outputlines.append(f"{geometry},{reso_method},{traffic_level},"
                               + f"{scenario},{confpair},{duration},{is_los},"
                               + f"{los_severity:0.4f},{start},{end},"
                               + f"{delta_v},{delta_trk}")

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "confpair,conflict duration [s],is LoS [-],"
                       + "LoS severity [-],t start [s],t end [s],delta v [kts],"
                       + "delta trk [deg]")


class ASASPosLocationParser(LogListParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and write all coordinates at which aircraft are in
    conflict to 'output_file'.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        outputlines = []
        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, is_los, lat, lon] = row
            simt = int(simt)
            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            lat = float(lat)
            lon = float(lon)
            is_los = str(bool(int(is_los)))

            # Write lines for ac1 and ac2 at once
            outputlines.append(f"{geometry},{reso_method},{traffic_level},"
                               + f"{scenario},{lat:0.4f},{lon:0.4f},"
                               + f"{is_los}")

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac lat [deg],ac lon[deg],is LoS [-]")


class FLSTLogOccurrenceParser(LogListParser):
    """
    Parse and process the contents of all FLST_LOG_... files that are
    in 'input_list' and summarize the stats for each individual flight.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        outputlines = []
        for row in log_data:
            del_time = float(row[0])
            spawn_time = float(row[2])

            # Skip if outside logging interval
            if (spawn_time < T_LOG_INTERVAL_START
                    or del_time > T_LOG_INTERVAL_END):
                continue

            ac_id = row[1]
            flight_time = float(row[3])
            nominal_dist = float(row[4])
            actual_dist = float(row[5])
            dist_to_last_wp = float(row[6])
            time_in_conf = float(row[9])
            time_in_los = float(row[10])
            time_in_reso = float(row[11])
            num_tot_conf = int(row[12])
            num_tot_los = int(row[13])

            route_efficiency = nominal_dist / (actual_dist + dist_to_last_wp)
            perc_time_in_conf = time_in_conf / flight_time * 100
            perc_time_in_los = time_in_los / flight_time * 100
            perc_time_in_reso = time_in_reso / flight_time * 100

            line = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                    + f"{ac_id},{route_efficiency:0.3f},"
                    + f"{perc_time_in_conf:0.3f},{perc_time_in_los:0.3f},"
                    + f"{perc_time_in_reso:0.3f},{num_tot_conf},{num_tot_los}")
            outputlines.append(line)

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac id,route efficiency [-],t in conf [%],t in los [%],"
                       + "t in reso [%],num tot conf [-],num tot los [-]")


class FLSTLogSummaryParser(LogListParser):
    """
    Parse and process the contents of all FLST_LOG_... files that are
    in 'input_list' and summarize the stats for each scenario.
    """

    def summarize_stats(self,
                        logfile,
                        log_data,
                        geometry,
                        reso_method,
                        scenario,
                        traffic_level):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        num_turnaround = 0

        for row in log_data:
            del_time = float(row[0])
            spawn_time = float(row[2])
            if (spawn_time < T_LOG_INTERVAL_START
                    or del_time > T_LOG_INTERVAL_END):
                continue

            lat = float(row[7])

            if lat < 0:
                num_turnaround += 1

        outputline = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                      + f"{num_turnaround}")
        self.write_to_output_file(outputline)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "num turnaround [-]")
