"""
Generate figures showing conflict locations.
"""

# Python imports
import os
import sys
import csv
import random

# Third-party imports
import pandas as pd
import seaborn as sbn
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join("..")))

# BlueSky imports
import bluesky.tools.geo as bsgeo


def make_batch_figures(timestamp):
    """
    Generate the figures for the batch with given timestamp.
    """

    GeoFigureGenerator(timestamp)
    BoxPlotFigureGenerator(timestamp)
    ViolinPlotFigureGenerator(timestamp)
    StripPlotFigureGenerator(timestamp)


def load_csv_data(filename, delimiter=",", comment_token="#"):
    """
    Loads the csv data from filename and returns a list that contains lists
    of row elements in string format.
    """

    with open(filename) as csv_file:
        reader = csv.reader(csv_file, delimiter=delimiter)
        data = [row for row in list(reader)
                if not row[0].startswith(comment_token)]

    return data


class FigureGeneratorBase:
    """
    Base class that defines the directory in which all figures will be saved
    for a given batch. Also creates this directory if it does not exist yet.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp
        self.combination_dict = {}

        self.batch_dir = os.path.join("post_processing", self.timestamp)
        self.figure_dir = os.path.join(self.batch_dir, "figures")
        self.create_figure_dir_if_not_exists()
        self.make_combination_dict()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the directory in which all figures will be saved is
        created if it does not exist yet.
        """

        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def make_combination_dict(self):
        """
        Create a nested dictionary that contains a list of all scenarios runs
        for each geometry, resolution method, and traffic level combination
        that is present in this batch.
        """

        combi_file_name = os.path.join("scenario",
                                       self.timestamp,
                                       "combinations.csv")
        combination_list = load_csv_data(combi_file_name)
        combi_dict = {}

        for combination in combination_list:
            # Get the values of the parameters in the current combination
            [scenname, length, width, angle, method, level] = combination
            geometry = f"L{length}_W{width}_A{angle}"

            # Make sure that dictionary elements exist before
            # appending the scenario name
            if not geometry in combi_dict.keys():
                combi_dict[geometry] = {}
            if not method in combi_dict[geometry].keys():
                combi_dict[geometry][method] = {}
            if not level in combi_dict[geometry][method].keys():
                combi_dict[geometry][method][level] = []
            combi_dict[geometry][method][level].append(scenname)

        self.combination_dict = combi_dict


class GeoFigureGenerator(FigureGeneratorBase):
    """
    Generates the geographic figures for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)

        self.load_conflict_location_data()
        self.generate_geo_figures()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all geo figures will be
        saved is created in case it does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "geo")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def load_conflict_location_data(self):
        """
        Load the locations of all conflicts from file.
        """

        summary_dir = os.path.join(self.batch_dir, "logfiles_summary")
        asaslog_file = os.path.join(summary_dir, "asaslog_locations.csv")
        self.asas_location_list = load_csv_data(asaslog_file)

    def generate_geo_figures(self):
        """
        Creates the figures showing the geographic features of the scenario
        as well as the locations of the conflicts and losses of separation
        per separation method.
        """

        for geometry in self.combination_dict:
            # Load the data for the current geometry
            geo_source_file_name = f"{self.timestamp}_{geometry}_geo.csv"
            geo_source_file = os.path.join("scenario",
                                           self.timestamp,
                                           geo_source_file_name)
            geo_data = load_csv_data(geo_source_file)

            # Create and save the base plot with geometry only
            geo_plot = self.make_geo_base_figure(geo_data)
            geo_plot_filename = os.path.join(self.figure_dir,
                                             f"{geometry}.png")
            geo_plot.savefig(geo_plot_filename, dpi=600)
            geo_plot.close()

            # Create the plots showing the conflict and intrusion locations
            for method in self.combination_dict[geometry]:
                for level in self.combination_dict[geometry][method]:
                    self.make_geo_location_figure(geo_data,
                                                  geometry,
                                                  method,
                                                  level,
                                                  location_type="conflict")
                    self.make_geo_location_figure(geo_data,
                                                  geometry,
                                                  method,
                                                  level,
                                                  location_type="intrusion")
                    self.make_geo_location_figure(geo_data,
                                                  geometry,
                                                  method,
                                                  level,
                                                  location_type="both")

    def make_geo_base_figure(self, geo_data):
        """
        Plot the experiment area and airspace restrictions defined in 'geo_data'.

        Returns a matplotlib.pyplot object that can then be used by the caller.
        """

        # Generate ring polygon
        ring_data = geo_data[0]
        ring_name = ring_data[0]
        ring_lat = float(ring_data[1])
        ring_lon = float(ring_data[2])
        ring_radius = float(ring_data[3])
        ring_coords = [bsgeo.qdrpos(ring_lat, ring_lon, angle, ring_radius)
                       for angle in range(0, 361)]
        ring_polygon = Polygon(ring_coords)

        # Generate polygon for restriction on left side of corridor
        left_res_name = geo_data[1][0]
        left_res_coords = [(float(lon), float(lat)) for (lat, lon)
                           in zip(geo_data[1][1:8:2], geo_data[1][2:9:2])]
        left_res_polygon = Polygon(left_res_coords)

        # Generate polygon for restriction on right side of corridor
        right_res_name = geo_data[2][0]
        right_res_coords = [(float(lon), float(lat)) for (lat, lon)
                            in zip(geo_data[2][1:8:2], geo_data[2][2:9:2])]
        right_res_polygon = Polygon(right_res_coords)

        # Calculate the intersections of the restriction polygons
        # and the experiment area ring polygon
        left_res_in_ring = ring_polygon.intersection(left_res_polygon)
        right_res_in_ring = ring_polygon.intersection(right_res_polygon)

        # Create the actual plot
        plt.figure()
        for area in [left_res_in_ring, right_res_in_ring]:
            plt.fill(*area.exterior.xy,
                     facecolor="xkcd:pale pink",
                     edgecolor="xkcd:brick red",
                     linewidth=1,
                     label="_nolegend_")
        plt.plot(*ring_polygon.exterior.xy,
                 "k",
                 linewidth=1,
                 label="_nolegend_")
        plt.axis("scaled")
        plt.xlabel("longitude [deg]")
        plt.ylabel("latitude [deg]")

        return plt

    def make_geo_location_figure(self,
                                 geo_data,
                                 geometry,
                                 separation_method,
                                 traffic_level,
                                 location_type):
        """
        Generate a figure that overlays a set of location markers onto
        the base figure for that geometry.

        The types of locations that can be shown is determined by
        'location_type' and can take the values: "conflict",
        "intrusion", or "both".
        """

        # Store conflict and intrusion locations as list of lists
        # using the format [[lon0, lon1, ...], [lat0, lat1, ...]]
        conflict_locations = [[], []]
        intrusion_locations = [[], []]
        for row in self.asas_location_list:
            if row[0:3] == [geometry, separation_method, traffic_level]:
                [ac_lat, ac_lon, is_los] = row[4:7]
                if is_los == "True":
                    intrusion_locations[0].append(float(ac_lon))
                    intrusion_locations[1].append(float(ac_lat))
                else:
                    conflict_locations[0].append(float(ac_lon))
                    conflict_locations[1].append(float(ac_lat))

        # Plot the locations on top of the geometry and save the figure
        plt = self.make_geo_base_figure(geo_data)
        alpha_plt = 0.02
        marker_plt = "."
        if location_type in ["conflict", "both"]:
            plt.scatter(conflict_locations[0],
                        conflict_locations[1],
                        alpha=alpha_plt,
                        marker=marker_plt,
                        edgecolors="none",
                        c="blue",
                        label="Conflict")
        if location_type in ["intrusion", "both"]:
            plt.scatter(intrusion_locations[0],
                        intrusion_locations[1],
                        alpha=alpha_plt,
                        marker=marker_plt,
                        edgecolors="none",
                        c="red",
                        label="Loss of separation")
        plt.title(f"Separation method: {separation_method}")
        # plt.legend()
        plt_filename = (f"{geometry}_{separation_method}_{traffic_level}"
                        + f"_{location_type}.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()


class BoxPlotFigureGenerator(FigureGeneratorBase):
    """
    Generate the box plots for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.generate_boxplot_figures()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all box plot figures will
        be saved is created in case does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "boxplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def generate_boxplot_figures(self):
        """
        Generate all box plot figures for each geometry in the batch.
        """

        for geometry in self.combination_dict:
            # Make figures based on data in asaslog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "conflict duration [s]",
                                    "conflict_duration")

            df_LoS_sev = df[(df["#geometry"] == geometry)
                            & (df["is LoS [-]"] == True)]
            self.make_single_figure(geometry,
                                    df_LoS_sev,
                                    "conflict duration [s]",
                                    "conflict_duration")

            # Make figures based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num conflicts [-]",
                                    "num_conflicts")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num LoS [-]",
                                    "num_LoS")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "IPR [-]",
                                    "IPR")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "DEP [-]",
                                    "DEP")

            # Make figures based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            df_destreached = df[(df["#geometry"] == geometry)
                                & (df["dist to last wp [NM]"] <= 0.5)]
            df_destnotreached = df[(df["#geometry"] == geometry)
                                   & (df["dist to last wp [NM]"] > 0.5)]

            self.make_single_figure(geometry,
                                    df_geometry,
                                    "work [GJ]",
                                    "work")
            self.make_single_figure(geometry,
                                    df_destreached,
                                    "work [GJ]",
                                    "work_destreached")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "route efficiency [-]",
                                    "efficiency")
            self.make_single_figure(geometry,
                                    df_destreached,
                                    "route efficiency [-]",
                                    "efficiency_destreached")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "dist to last wp [NM]",
                                    "dist_to_last")
            # self.make_single_figure(geometry,
            #                         df_destreached,
            #                         "dist to last wp [NM]",
            #                         "dist_to_last_destreached")
            self.make_single_figure(geometry,
                                    df_destnotreached,
                                    "dist to last wp [NM]",
                                    "dist_to_last_destnotreached")

            # Make figures based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num dest not reached [-]",
                                    "num_destnotreached")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num turnaround [-]",
                                    "num_turnaround")

    def make_single_figure(self, geometry, df, column, namestr):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        # Use the min and max of the unfiltered column in the dataframe
        # to ensure all figures of this type have the same y-axis limits
        yrange = df[column].max() - df[column].min()
        ymin = df[column].min() - 0.05 * yrange
        ymax = df[column].max() + 0.05 * yrange

        plt.figure()
        ax = sbn.boxplot(x="resolution method",
                         y=column,
                         data=df,
                         order=["OFF", "MVP", "LF", "SWARM-V2"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"],
                         palette="Blues",
                         linewidth=0.5)
        plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 2))
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        ax.set(xticklabels=["OFF", "MVP", "MVP+LF", "MVP+SWARM"])
        ax.set_ylim(ymin, ymax)
        plt_filename = f"{geometry}_{namestr}.png"
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()


class ViolinPlotFigureGenerator(FigureGeneratorBase):
    """
    Generate the box plots for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.generate_violinplot_figures()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all violin plot figures
        will be saved is created in case does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "violinplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def generate_violinplot_figures(self):
        """
        Generate all box plot figures for each geometry in the batch.
        """

        for geometry in self.combination_dict:
            # Make figures based on data in asaslog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "conflict duration [s]",
                                    "conflict_duration")

            df_LoS_sev = df[(df["#geometry"] == geometry)
                            & (df["is LoS [-]"] == True)]
            self.make_single_figure(geometry,
                                    df_LoS_sev,
                                    "conflict duration [s]",
                                    "conflict_duration")

            # Make figures based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num conflicts [-]",
                                    "num_conflicts")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num LoS [-]",
                                    "num_LoS")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "IPR [-]",
                                    "IPR")

            # Make figures based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "work [GJ]",
                                    "work")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "route efficiency [-]",
                                    "efficiency")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "dist to last wp [NM]",
                                    "dist_to_last")

    def make_single_figure(self, geometry, df, column, namestr):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        plt.figure()
        ax = sbn.violinplot(x="resolution method",
                            y=column,
                            data=df,
                            order=["OFF", "MVP", "LF", "SWARM-V2"],
                            hue="traffic level",
                            hue_order=["LOW", "MID", "HIGH"],
                            palette="Blues",
                            linewidth=0.5)
        plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 2))
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = f"{geometry}_{namestr}.png"
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()


class StripPlotFigureGenerator(FigureGeneratorBase):
    """
    Generate the box plots for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.generate_stripplot_figures()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the directory in which all strip plot figures will
        be saved is created in case it does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "stripplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def generate_stripplot_figures(self):
        """
        Generate all box plot figures for each geometry in the batch.
        """

        for geometry in self.combination_dict:
            # Make figures based on data in asaslog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "conflict duration [s]",
                                    "conflict_duration")

            df_LoS_sev = df[(df["#geometry"] == geometry)
                            & (df["is LoS [-]"] == True)]
            self.make_single_figure(geometry,
                                    df_LoS_sev,
                                    "conflict duration [s]",
                                    "conflict_duration")

            # Make figures based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num conflicts [-]",
                                    "num_conflicts")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num LoS [-]",
                                    "num_LoS")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "IPR [-]",
                                    "IPR")

            # Make figures based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "work [GJ]",
                                    "work")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "route efficiency [-]",
                                    "efficiency")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "dist to last wp [NM]",
                                    "dist_to_last")

    def make_single_figure(self, geometry, df, column, namestr):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        plt.figure()
        ax = sbn.stripplot(x="resolution method",
                           y=column,
                           data=df,
                           order=["OFF", "MVP", "LF", "SWARM-V2"],
                           hue="traffic level",
                           hue_order=["LOW", "MID", "HIGH"],
                           palette="Blues",
                           linewidth=0.5,
                           jitter=True,
                           dodge=True)
        plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 2))
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = f"{geometry}_{namestr}.png"
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()


if __name__ == "__main__":
    # timestamp = "20190712-022110"
    # timestamp = "20190714-152439"
    timestamp = "20190717-015921"
    make_batch_figures(timestamp)
