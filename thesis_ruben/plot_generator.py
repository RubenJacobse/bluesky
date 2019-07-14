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
sys.path.append(os.path.abspath(os.path.join('..')))

# BlueSky imports
import bluesky.tools.geo as bsgeo


def make_batch_figures(timestamp):
    """
    Generate the figures for the batch with given timestamp.
    """

    GeoFigureGenerator(timestamp)
    BoxPlotFigureGenerator(timestamp)


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
                [ac_lat, ac_lon, is_los] = row[3:6]
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

    def generate_boxplot_figures(self):
        """
        Generate a box plot figure for each geometry.
        """

        for geometry in self.combination_dict:
            self.make_boxplot_figure(geometry)

    def make_boxplot_figure(self, geometry):
        """
        Make a single boxplot figure.
        """

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_occurence.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="conflict duration [s]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_conflict_duration.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_occurence.csv"))
        df_geometry = df[(df["#geometry"] == geometry) &
                         (df["is LoS [-]"] == True)]
        ax = sbn.boxplot(x="resolution method",
                         y="LoS severity [-]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_LoS_severity.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_summary.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="num conflicts [-]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_num_conflicts.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_summary.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="num LoS [-]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_num_LoS.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_summary.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="IPR [-]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_IPR.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "flstlog_occurence.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="work [J]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_work.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "flstlog_occurence.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="route efficiency [-]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_efficiency.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()

        plt.figure()
        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "flstlog_occurence.csv"))
        df_geometry = df[df["#geometry"] == geometry]
        ax = sbn.boxplot(x="resolution method",
                         y="dist to last wp [m]",
                         data=df_geometry,
                         order=["OFF", "MVP", "LF"],
                         hue="traffic level",
                         hue_order=["LOW", "MID", "HIGH"])
        ax.legend(loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.1))
        plt_filename = (f"{geometry}_disttolast.png")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=600)
        plt.close()


if __name__ == "__main__":
    # timestamp = "20190712-022110"
    timestamp = "20190714-152439"
    make_batch_figures(timestamp)
