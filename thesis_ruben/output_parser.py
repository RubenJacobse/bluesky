"""
Process the log output files written by the area restriction plugin
and
"""

# Python imports
import os
import csv
import sys
import statistics

# Third-party imports
import numpy as np
from shapely.geometry.polygon import Polygon

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))

# BlueSky imports
from bluesky import settings
settings.set_variable_defaults(log_start=1800,
                               log_end=9000)

# Module level constants
PZ_RADIUS = 5.0 * 1852  # Protected zone radius in meters
MPS_TO_KTS = 1.94384449  # Conversion factor from m/s to kts
R_EARTH = 6371000 # Approximate Earth radius in meters

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
                       + "scenario,num area conflicts [-],num area intrusions [-]")


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
                       + "ac lat [deg],ac lon [deg],is intrusion [-]")


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
        los_severity_list = []

        for row in log_data:
            # Last row of ASASLOG contains only time-averaged number
            # of aircraft in scenario during logging period
            if row[1] == "avg ntraf":
                avg_ntraf = float(row[2])
                break

            [simt, id1, id2, is_los, duration, dist_min, dcpa_min, tcpa_min,
             tcpa_init, tlos_init, dcpa_init, lat1, lon1, lat2, lon2,
             gseast1, gsnorth1, gseast2, gsnorth2] = row
            simt = int(float(simt))
            is_los = int(is_los)

            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            if is_los:
                num_los += 1
                los_severity = (PZ_RADIUS - float(dist_min)) / PZ_RADIUS
                los_severity_list.append(los_severity)
            else:
                num_conf += 1

        # Intrusion prevention rate not defined if baseline has zero
        # conflicts (divide by zero)
        if num_conf:
            int_prev_rate = f"{(num_conf - num_los) / num_conf:.3f}"
        else:
            int_prev_rate = "NaN"

        # Calculate summarizing statistics for entire run
        if los_severity_list:
            los_sev_stat = f'{statistics.mean(los_severity_list):.4f}'
        else:
            los_sev_stat = "NaN"

        # Calculate area available to traffic
        timestamp = logfile.split(os.sep)[1]
        geomfile_dir = os.path.join(*logfile.split(os.sep)[:2],
                                    "geomfiles" + os.sep)
        geom_csv_file = os.path.join(geomfile_dir,
                                     f"{timestamp}_{geometry}_geo.csv")
        area_list = []
        with open(geom_csv_file) as geom_csv:
            # Calculate the areas of each polygon in NM^2
            for line in geom_csv:
                coords = line[:-1].split(",")[1:]
                coord_pairs = [(float(lat), float(lon)) for (lat, lon)
                               in zip(coords[0::2], coords[1::2])]
                coord_pairs_rad = [(np.radians(lat), np.radians(lon))
                                   for (lat, lon) in coord_pairs]
                # Next line assumes flat Earth and polygon near lat, lon = 0,0
                metric_pairs = [(R_EARTH * lat, R_EARTH * lon * np.cos(lat))
                                for (lat, lon) in coord_pairs_rad]
                poly = Polygon(metric_pairs)
                area_list.append(poly.area / 1852**2)  # convert to NM^2
        traf_area = area_list[0] - sum(area_list[1:])

        current_stats = {"geometry": geometry,
                         "reso_method": reso_method,
                         "traffic_level": traffic_level,
                         "scenario": scenario,
                         "num_conf": num_conf,
                         "num_los": num_los,
                         "int_prev_rate": int_prev_rate,
                         "avg_ntraf": avg_ntraf,
                         "traf_area": traf_area,
                         "los_sev_stat": los_sev_stat}

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

                # Domino Effect Parameter not defined if baseline has
                # zero conflicts (divide by zero)
                if basescen["num_conf"]:
                    scen_dep = (scen["num_conf"] / basescen["num_conf"]) - 1
                    scen["dep"] = f"{scen_dep:.3f}"
                else:
                    scen["dep"] = "NaN"

            outputline = (f'{scen["geometry"]},{scen["reso_method"]},'
                          + f'{scen["traffic_level"]},{scen["scenario"]},'
                          + f'{scen["num_conf"]},{scen["num_los"]},'
                          + f'{scen["int_prev_rate"]},{scen["dep"]},'
                          + f'{scen["avg_ntraf"]:.2f},'
                          + f'{1e4 * scen["avg_ntraf"]/scen["traf_area"]:.4f},'
                          + f'{scen["los_sev_stat"]}')
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
                       + "num conflicts [-],num LoS [-],IPR [-],DEP [-],"
                       + "avg num ac [-],avg density [ac/1e4NM^2],"
                       + "LoS sev stat [-]")


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
            # Last row of ASASLOG contains only time-averaged number
            # of aircraft in scenario during logging period
            if row[1] == "avg ntraf":
                break

            [simt, id1, id2, is_los, duration, dist_min, dcpa_min, tcpa_min,
             tcpa_init, tlos_init, dcpa_init, lat1, lon1, lat2, lon2,
             gseast1, gsnorth1, gseast2, gsnorth2] = row

            simt = int(float(simt))
            if simt < T_LOG_INTERVAL_START or simt > T_LOG_INTERVAL_END:
                continue

            confpair = f"{id1}-{id2}"
            is_los = int(is_los)
            los_severity = (PZ_RADIUS - float(dist_min)) / PZ_RADIUS
            duration = int(duration)
            start = simt - duration
            end = simt
            gseast1 = float(gseast1)
            gsnorth1 = float(gsnorth1)
            gseast2 = float(gseast2)
            gsnorth2 = float(gsnorth2)

            # Relative velocity and track at start of conflict/los
            delta_v = [gseast1 - gseast2, gsnorth1 - gsnorth2]
            delta_v_abs = np.linalg.norm(delta_v) * MPS_TO_KTS
            trk1 = np.degrees(np.arctan2(gseast1, gsnorth1)) % 360
            trk2 = np.degrees(np.arctan2(gseast2, gsnorth2)) % 360
            delta_trk = abs(crs_diff(trk1, trk2))

            outputlines.append(f"{geometry},{reso_method},{traffic_level},"
                               + f"{scenario},{confpair},{duration},{is_los},"
                               + f"{los_severity:0.4f},{start},{end},"
                               + f"{delta_v_abs:.1f},{delta_trk:.1f}")

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "confpair,duration [s],is LoS [-],"
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
            is_los = int(is_los)

            # Write lines for ac1 and ac2 at once
            outputlines.append(f"{geometry},{reso_method},{traffic_level},"
                               + f"{scenario},{lat:0.4f},{lon:0.4f},"
                               + f"{is_los}")

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac lat [deg],ac lon [deg],is LoS [-]")


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
            ac_id = row[1]
            spawn_time = float(row[2])

            # Skip if outside logging interval
            if (spawn_time < T_LOG_INTERVAL_START
                    or del_time > T_LOG_INTERVAL_END):
                continue

            flight_time = float(row[3])
            #extrap_time = float(row[4]) # total to dest
            nominal_dist = float(row[5])
            actual_dist = float(row[6])
            dist_to_last_wp = float(row[7])
            # lat = float(row[8])
            # lon = float(row[9])
            time_in_conf = float(row[10])
            time_in_los = float(row[11])
            time_in_reso = float(row[12])
            num_tot_conf = float(row[13])
            num_tot_los = float(row[14])
            num_tot_area_conf = int(row[15])
            num_tot_area_int = int(row[16])
            time_in_area_conf = float(row[17])
            time_in_area_int = float(row[18])

            route_efficiency = nominal_dist / (actual_dist + dist_to_last_wp)
            perc_time_in_conf = time_in_conf / flight_time * 100
            perc_time_in_los = time_in_los / flight_time * 100
            perc_time_in_reso = time_in_reso / flight_time * 100
            perc_time_in_area_conf = time_in_area_conf / flight_time * 100
            perc_time_in_area_int = time_in_area_int / flight_time * 100

            line = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                    + f"{ac_id},{route_efficiency:0.3f},"
                    + f"{perc_time_in_conf:0.3f},{perc_time_in_los:0.3f},"
                    + f"{perc_time_in_reso:0.3f},{num_tot_conf},{num_tot_los},"
                    + f"{num_tot_conf/actual_dist:0.10f},"
                    + f"{perc_time_in_area_conf:0.3f},"
                    + f"{perc_time_in_area_int:0.3f},"
                    + f"{num_tot_area_conf},{num_tot_area_int}")
            outputlines.append(line)

        self.write_lines_to_output_file(outputlines)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "ac id,route efficiency [-],t in conf [%],t in los [%],"
                       + "t in reso [%],num conf per ac [-],num los per ac [-],"
                       + "num ac conf per/ac/dist [1/m],t in area conf [%],"
                       + "t in area intrusion [%],num area conf per ac [-],"
                       + "num area int per ac [-]")


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
        extra_dist_list = []
        t_conf_list = []
        t_los_list = []
        t_reso_list = []
        ac_conf_per_dist_list = []

        for row in log_data:
            del_time = float(row[0])
            spawn_time = float(row[2])
            if (spawn_time < T_LOG_INTERVAL_START
                    or del_time > T_LOG_INTERVAL_END):
                continue

            flight_time = float(row[3])
            lat = float(row[8])
            nominal_dist = float(row[5])
            actual_dist = float(row[6])
            dist_to_last_wp = float(row[7])
            time_in_conf = float(row[10])
            time_in_los = float(row[11])
            time_in_reso = float(row[12])
            num_tot_conf = float(row[13])
            if lat < 0:
                num_turnaround += 1
            else:
                extra_dist = 100 * (actual_dist + dist_to_last_wp - nominal_dist) / nominal_dist
                extra_dist_list.append(extra_dist)

            t_conf_list.append(time_in_conf / flight_time * 100)
            t_los_list.append(time_in_los / flight_time * 100)
            t_reso_list.append(time_in_reso / flight_time * 100)
            ac_conf_per_dist_list.append(num_tot_conf / actual_dist)

        #
        extra_dist_stat = statistics.mean(extra_dist_list)
        t_conf_stat = statistics.mean(t_conf_list)
        t_los_stat = statistics.mean(t_los_list)
        t_reso_stat = statistics.mean(t_reso_list)
        ac_conf_per_dist_stat = statistics.mean(ac_conf_per_dist_list)

        outputline = (f"{geometry},{reso_method},{traffic_level},{scenario},"
                      + f"{num_turnaround},{extra_dist_stat},{t_conf_stat},"
                      + f"{t_los_stat},{t_reso_stat},"
                      + f"{ac_conf_per_dist_stat:.10f}")
        self.write_to_output_file(outputline)

    def set_header(self):
        self.header = ("geometry,resolution method,traffic level,scenario,"
                       + "num turnaround [-],extra dist [%],time in conflict [%],"
                       + "time in los [%],time in resolution [%],"
                       + "avg ac conf per dist [1/m]")


def crs_diff(crs_a, crs_b):
    """
    Returns the absolute difference per element between two course vectors crs_a
    and crs_b.
    """

    # Calculate absolute angle difference between both courses and the reference
    diff = np.remainder(crs_b - crs_a + 180, 360) - 180

    return diff
