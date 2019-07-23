"""
Process the log output files written by the area restriction plugin
and
"""

# Python imports
import csv

# Module level constants
PZ_RADIUS = 5.0 * 1852  # Protected zone radius in meters


class LogListParser:
    """
    Parse and process the contents of all log files that are
    in 'input_list'.
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
            if not t_int == 0:
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

        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, ac_id, area_id, t_int, ac_lat, ac_lon] = row
            simt = int(float(simt))
            t_int = float(t_int)
            ac_lat = float(ac_lat)
            ac_lon = float(ac_lon)

            is_int = t_int == 0

            # Write lines for ac1 and ac2 at once
            outputlines = (f"{geometry},{reso_method},{traffic_level},"
                           + f"{scenario},{ac_lat:0.6f},{ac_lon:0.6f},"
                           + f"{is_int}")
            self.write_to_output_file(outputlines)

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
        conf_dict = {}
        los_dict = {}

        for row in log_data:
            [simt, ac1_id, ac2_id, dist, t_cpa, t_los,
             ac1_lat, ac1_lon, ac2_lat, ac2_lon] = row
            simt = int(float(simt))
            dist = float(dist)
            t_cpa = float(t_cpa)
            t_los = float(t_los)
            ac1_lat = float(ac1_lat)
            ac1_lon = float(ac1_lon)
            ac2_lat = float(ac2_lat)
            ac2_lon = float(ac2_lon)

            confpair = f"{ac1_id}-{ac2_id}"

            if confpair not in conf_dict.keys():
                conf_dict[confpair] = []

            # Check if intrusion count needs to be incremented
            if conf_dict[confpair]:
                if not conf_dict[confpair][-1] == simt-1:
                    num_conf += 1
            else:
                num_conf += 1
            conf_dict[confpair].append(simt)

            # Continue to next occurence if no loss of separation
            if not dist < PZ_RADIUS:
                continue

            # Check if los count needs to be incremented
            if confpair not in los_dict.keys():
                los_dict[confpair] = []
            if los_dict[confpair]:
                if not los_dict[confpair][-1] == simt-1:
                    num_los += 1
            else:
                num_los += 1
            los_dict[confpair].append(simt)

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

        conf_dict = {}

        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, ac1_id, ac2_id, dist, t_cpa, t_los,
             ac1_lat, ac1_lon, ac2_lat, ac2_lon] = row
            simt = int(float(simt))
            dist = float(dist)
            t_cpa = float(t_cpa)
            t_los = float(t_los)
            ac1_lat = float(ac1_lat)
            ac1_lon = float(ac1_lon)
            ac2_lat = float(ac2_lat)
            ac2_lon = float(ac2_lon)

            confpair = f"{ac1_id}-{ac2_id}"
            is_los = dist <= PZ_RADIUS
            los_severity = (PZ_RADIUS - dist) / PZ_RADIUS

            new_conflict = {"start_time": simt,
                            "end_time": simt + 1,
                            "duration": 1,
                            "is_los": is_los,
                            "los_severity": los_severity}

            if confpair not in conf_dict.keys():
                # New conflict pair
                conf_dict[confpair] = [new_conflict]
            elif conf_dict[confpair][-1]["end_time"] == simt:
                # Continuing conflict, intrement time 'counters'
                conf_dict[confpair][-1]["end_time"] += 1
                conf_dict[confpair][-1]["duration"] += 1

                # Check for LoS of separation (once set to True will always
                # remain True)
                conf_dict[confpair][-1]["is_los"] = (
                    conf_dict[confpair][-1]["is_los"] or is_los)

                # Reset severity if worse than current worst
                if (conf_dict[confpair][-1]["is_los"]
                        and conf_dict[confpair][-1]["los_severity"] < los_severity):
                    conf_dict[confpair][-1]["los_severity"] = los_severity
            else:
                # Add new conflict for this confpair
                conf_dict[confpair].append(new_conflict)

        # Write all conflicts to file
        for confpair in conf_dict:
            for conflict in conf_dict[confpair]:
                duration = conflict["duration"]
                los_severity = conflict["los_severity"]
                start = conflict["start_time"]
                end = conflict["end_time"]
                is_los = conflict["is_los"]

                outputline = (f"{geometry},{reso_method},{traffic_level},"
                              + f"{scenario},{confpair},{duration},{is_los},"
                              + f"{los_severity:0.3f},{start},{end}")
                self.write_to_output_file(outputline)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "confpair,conflict duration [s],is LoS [-],"
                       + "LoS severity [-],t start [s],t end[s]")


class ASASLogLocationParser(LogListParser):
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

        # Loop over all rows and create a dictionary with each conflict
        # and its parameters listed once
        for row in log_data:
            [simt, ac1_id, ac2_id, dist, t_cpa, t_los,
             ac1_lat, ac1_lon, ac2_lat, ac2_lon] = row
            simt = int(float(simt))
            dist = float(dist)
            t_cpa = float(t_cpa)
            t_los = float(t_los)
            ac1_lat = float(ac1_lat)
            ac1_lon = float(ac1_lon)
            ac2_lat = float(ac2_lat)
            ac2_lon = float(ac2_lon)

            is_los = dist <= PZ_RADIUS

            # Write lines for ac1 and ac2 at once
            outputlines = (f"{geometry},{reso_method},{traffic_level},"
                           + f"{scenario},{ac1_lat:0.6f},{ac1_lon:0.6f},"
                           + f"{is_los}\n"
                           + f"{geometry},{reso_method},{traffic_level},"
                           + f"{scenario},{ac2_lat:0.6f},{ac2_lon:0.6f},"
                           + f"{is_los}")
            self.write_to_output_file(outputlines)

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

        for row in log_data:
            ac_id = row[1]
            nominal_dist = float(row[4])
            actual_dist = float(row[5])
            work_performed = float(row[7]) / 1e9
            dist_to_last_wp = float(row[8]) / 1852

            route_efficiency = nominal_dist / actual_dist

            line = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                    + f"{ac_id},{work_performed:0.0f},{route_efficiency:0.3f},"
                    + f"{dist_to_last_wp:0.1f}")
            self.write_to_output_file(line)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac id,work [GJ],route efficiency [-],"
                       + "dist to last wp [NM]")


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

        num_dest_not_reached = 0
        num_turnaround = 0

        for row in log_data:
            dist_to_last_wp = float(row[8]) / 1852
            lat = float(row[9])

            if dist_to_last_wp > 0.5:
                num_dest_not_reached += 1

            if lat < 0:
                num_turnaround += 1

        outputline = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                      + f"{num_dest_not_reached},{num_turnaround}")
        self.write_to_output_file(outputline)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "num dest not reached [-],num turnaround [-]")
