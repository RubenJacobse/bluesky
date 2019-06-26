"""
Process the log output files written by the area restriction plugin
and
"""

# Python imports
import csv

# Module level constants
PZ_RADIUS = 5.0 * 1852


class LogParser:
    """
    Parse and process the contents of all log files that are
    in 'input_list'.
    """

    def __init__(self, input_list, output_file, output_header):
        self.input_list = input_list
        self.output_file = output_file

        # Create a new output file and write its header
        with open(self.output_file, "w") as output:
            output.write(output_header + "\n")

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
            # the summarized data to the output file
            self.summarize_stats(logfile, log_data)

    def write_to_output_file(self, line):
        """
        Write a line containing the relevant parameters of the current
        logfile to the output file.
        """

        # Output file already exists, thus we append to it
        with open(self.output_file, "a") as output:
            output.write(line + "\n")

    # Function that needs to be overridden in the actual implementation
    # classes
    def summarize_stats(self, logfile, log_data):
        pass


class AREALogSummaryParser(LogParser):
    """
    Parse and process the contents of all AREA_LOG_... files that are
    in 'input_list'.
    """

    def summarize_stats(self, logfile, log_data):
        """
        Process the elements of the 'log_data' list and write the
        summary to logfile.
        """

        num_intrusions = 0
        intrusion_dict = {}

        for row in log_data:
            [simt, ac_id, area_id, t_int, ac_lat, ac_lon] = row
            simt = int(float(simt))
            t_int = float(t_int)
            ac_lat = float(ac_lat)
            ac_lon = float(ac_lon)

            confpair = f"{ac_id}-{area_id}"

            if t_int > 1e-9:
                continue

            if confpair not in intrusion_dict.keys():
                intrusion_dict[confpair] = []

            # Check if intrusion count needs to be incremented
            if intrusion_dict[confpair]:
                if not intrusion_dict[confpair][-1] == simt-1:
                    num_intrusions += 1
            else:
                num_intrusions += 1

            intrusion_dict[confpair].append(simt)

        outputline = f"{logfile},{num_intrusions}"
        self.write_to_output_file(outputline)


class ASASLogSummaryParser(LogParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and summarize the stats for each scenario.
    """

    def summarize_stats(self, logfile, log_data):
        """
        Process the elements of the 'log_data' list and write the
        summary to logfile.
        """

        num_los = 0
        num_conf = 0
        conf_dict = {}

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

            # Loss of separation only if
            if dist > PZ_RADIUS:
                continue

        int_prev_rate = (num_conf - num_los) / num_conf

        outputline = f"{logfile},{num_conf},{num_los},{int_prev_rate}"
        self.write_to_output_file(outputline)


class ASASLogOccurrenceParser(LogParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and summarize the stats for each individual conflict.
    """

    def summarize_stats(self, logfile, log_data):
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

                outputline = (f"{logfile},{confpair},{duration},{is_los}"
                              + f",{los_severity},{start},{end}")
                self.write_to_output_file(outputline)


class ASASLogLocationParser(LogParser):
    """
    Parse and process the contents of all ASAS_LOG_... files that are
    in 'input_list' and write all coordinates at which aircraft are in
    conflict to 'output_file'.
    """

    def summarize_stats(self, logfile, log_data):
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

            outputline = (f"{logfile},{ac1_lat},{ac1_lon},{is_los}\n"
                          + f"{logfile},{ac2_lat},{ac2_lon},{is_los}")
            self.write_to_output_file(outputline)


class FLSTLogOccurrenceParser(LogParser):
    """
    Parse and process the contents of all FLST_LOG_... files that are
    in 'input_list' and summarize the stats for each individual flight.
    """

    def summarize_stats(self, logfile, log_data):
        """
        Process the elements of the 'log_data' list and for each flight
        write the summary to 'logfile'.
        """

        for row in log_data:
            ac_id = row[1]
            nominal_dist = float(row[4])
            actual_dist = float(row[5])
            work_performed = float(row[7])
            dist_to_last_wp = float(row[8])

            route_efficiency = nominal_dist / actual_dist

            outputline = (f"{logfile},{ac_id},{work_performed},{route_efficiency},"
                          + f"{dist_to_last_wp}")
            self.write_to_output_file(outputline)


if __name__ == "__main__":
    arealog_list = [
        "test_output/AREA_CONF_LOG_20190619_224422_L40_W25_A90_RESO-SWARM_V2_SCEN002.log",
        "test_output/AREA_CONF_LOG_test.log"]
    AREALogSummaryParser(arealog_list,
                         "arealog_summary.csv",
                         "logfile, num intrusions [-]")

    asaslog_list = [
        "test_output/ASAS_CONF_LOG_20190619_224422_L40_W25_A90_RESO-SWARM_V2_SCEN002.log",
        "test_output/ASAS_CONF_LOG_test.log"]
    ASASLogSummaryParser(asaslog_list,
                         "asaslog_summary.csv",
                         "logfile, num conflicts [-], num LoS [-], IPR [-]")
    ASASLogOccurrenceParser(asaslog_list,
                            "asaslog_occurence.csv",
                            "logfile, confpair, conflict duration [s], LoS severity [-]")
    ASASLogLocationParser(asaslog_list,
                          "asaslog_locations.csv",
                          "logfile, ac lat [deg], ac lon[deg], is LoS [-]")

    flstlog_list = [
        "test_output/FLSTLOG_20190619_224422_L40_W25_A90_RESO-SWARM_V2_SCEN002.log"]
    FLSTLogOccurrenceParser(flstlog_list,
                            "flstlog_occurence.csv",
                            "logfile, ac id, work [J], route efficiency [-], dist to last wp [m]")
