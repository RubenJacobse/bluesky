"""
Generate figures showing conflict locations.
"""

# Python imports
import os
import sys
import csv

# Third-party imports
import numpy as np
import pandas as pd
import seaborn as sbn
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from shapely.geometry import Polygon

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join("..")))

# BlueSky imports
import bluesky.tools.geo as bsgeo

# Local imports
# import extra_plot

# Geo plot element colors
RESTRICTION_FACECOLOR = "xkcd:pale pink"
RESTRICTION_EDGECOLOR = "xkcd:brick red"
GEOVECTOR_EDGECOLOR = "xkcd:boring green"
GEOVECTOR_FACECOLOR = "xkcd:light seafoam green"
INTRUSION_MARKER_COLOR = "red"
CONFLICT_MARKER_COLOR = "blue"

# Save figures as following type
FIGURE_FILETYPE = "png"


def make_batch_figures(timestamp):
    """
    Generate the figures for the batch with given timestamp.
    """

    # BoxPlotFigureGenerator(timestamp)
    # AREAGeoFigureGenerator(timestamp)
    # ASASGeoFigureGenerator(timestamp)
    ASASConflictFigureGenerator(timestamp)
    # ViolinPlotFigureGenerator(timestamp)
    # StripPlotFigureGenerator(timestamp)


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
        for each {geometry, resolution method, traffic level} combination
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


class GeoFigureGeneratorBase(FigureGeneratorBase):
    """
    Abstract base class that contains part of the logic to generate
    the geographic figures for a batch with given timestamp. Actual
    implementation is done via subclasses in which the source of the
    location data is specified.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)

        self.load_conflict_location_data()
        self.generate_geo_figures()

    def load_conflict_location_data(self):
        """
        Load the locations of all conflicts from file.
        """

        raise NotImplementedError

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
                                             f"{geometry}.{FIGURE_FILETYPE}")
            geo_plot.savefig(geo_plot_filename, dpi=300, bbox_inches="tight")
            geo_plot.close()

            # Create the plots showing the conflict and intrusion locations
            for method in self.combination_dict[geometry]:
                for level in self.combination_dict[geometry][method]:
                    # Make area conflict figures. Valid location_type
                    # settings: "conflict", "intrusion", "both"
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
                     facecolor=RESTRICTION_FACECOLOR,
                     edgecolor=RESTRICTION_EDGECOLOR,
                     linewidth=1,
                     label="_nolegend_",
                     zorder=0)
        plt.plot(*ring_polygon.exterior.xy,
                 "k",
                 linewidth=1,
                 label="_nolegend_",
                 zorder=0)
        plt.axis("scaled")
        plt.xlabel("longitude [deg]")
        plt.ylabel("latitude [deg]")

        return plt

    def make_geovec_figure(self, geo_data, geovec_data):
        """
        Plot the experiment area and airspace restrictions defined in
        'geo_data' together with the geovector areas defined in 'geovec_data'.

        Returns a matplotlib.pyplot object that can then be used by the caller.
        """

        # Create the base figure with experiment area and airspace restrictions
        plt = self.make_geo_base_figure(geo_data)

        # Add each geovector area to the figure
        for row in geovec_data:
            gv_name = row[0]
            gv_gsmin = row[1]
            gv_gsmax = row[2]
            gv_trkmin = row[3]
            gv_trkmax = row[4]
            gv_coords = row[5:]

            gv_latlon = [(float(lon), float(lat)) for (lat, lon)
                         in zip(gv_coords[0:-1:2], gv_coords[1::2])]
            gv_polygon = Polygon(gv_latlon)

            plt.fill(*gv_polygon.exterior.xy,
                     edgecolor=GEOVECTOR_EDGECOLOR,
                     facecolor=GEOVECTOR_FACECOLOR,
                     linewidth=1,
                     label="_nolegend_",
                     zorder=0)

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

        # Create list of elements to display in the figure legend
        legend_elements = [Patch(facecolor=RESTRICTION_FACECOLOR,
                                 edgecolor=RESTRICTION_EDGECOLOR,
                                 linewidth=1,
                                 label="Restricted area")]

        # Store conflict and intrusion locations as list of lists
        # using the format [[lon0, lon1, ...], [lat0, lat1, ...]]
        conflict_locations = [[], []]
        intrusion_locations = [[], []]
        for row in self.location_list:
            if row[0:3] == [geometry, separation_method, traffic_level]:
                [ac_lat, ac_lon, is_los] = row[4:7]
                if is_los == "True":
                    intrusion_locations[0].append(float(ac_lon))
                    intrusion_locations[1].append(float(ac_lat))
                else:
                    conflict_locations[0].append(float(ac_lon))
                    conflict_locations[1].append(float(ac_lat))

        # Create the base geometry plot
        if "GV-" in separation_method:
            # Load the geovector data for the current geometry
            geovec_source_file_name = (f"{self.timestamp}_{geometry}_RESO-"
                                       + f"{separation_method}_geovector.csv")
            geovec_source_file = os.path.join("scenario",
                                              self.timestamp,
                                              geovec_source_file_name)
            geovec_data = load_csv_data(geovec_source_file)

            # Create plot with restriction and geovector areas
            plt = self.make_geovec_figure(geo_data, geovec_data)

            # Add geovector to legend
            legend_elements.append(Patch(edgecolor=GEOVECTOR_EDGECOLOR,
                                         facecolor=GEOVECTOR_FACECOLOR,
                                         linewidth=1,
                                         label="Geovectoring area"))
        else:
            # Create plot with only restriction areas
            plt = self.make_geo_base_figure(geo_data)

        # Add the location markers to the figure
        alpha_plt = 0.01
        marker_plt = "."
        if location_type in ["conflict", "both"]:
            plt.scatter(conflict_locations[0],
                        conflict_locations[1],
                        alpha=alpha_plt,
                        marker=marker_plt,
                        edgecolors="none",
                        c=CONFLICT_MARKER_COLOR,
                        label="_nolegend_")
            legend_elements.append(Line2D([0], [0],
                                          linestyle="",
                                          marker=marker_plt,
                                          color=CONFLICT_MARKER_COLOR,
                                          label="Conflict locations"))
        if location_type in ["intrusion", "both"]:
            plt.scatter(intrusion_locations[0],
                        intrusion_locations[1],
                        alpha=alpha_plt,
                        marker=marker_plt,
                        edgecolors="none",
                        c=INTRUSION_MARKER_COLOR,
                        label="_nolegend_")
            legend_elements.append(Line2D([0], [0],
                                          linestyle="",
                                          marker=marker_plt,
                                          color=INTRUSION_MARKER_COLOR,
                                          label="LoS locations"))
        # plt.title(f"Separation method: {separation_method}")
        plt.legend(handles=legend_elements, loc="lower center",
                   ncol=2, bbox_to_anchor=(0.5, 1))
        plt_filename = (f"{geometry}_{separation_method}_{traffic_level}"
                        + f"_{location_type}.{FIGURE_FILETYPE}")
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
        plt.close()


class ASASGeoFigureGenerator(GeoFigureGeneratorBase):
    """
    Generates the geographic figures showing the aircraft conflicts
    and intrusions for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all geo figures will be
        saved is created in case it does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "geo", "asas")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def load_conflict_location_data(self):
        """
        Load the locations of all conflicts from file.
        """

        summary_dir = os.path.join(self.batch_dir, "logfiles_summary")
        asaslog_file = os.path.join(summary_dir, "asaslog_locations.csv")
        self.location_list = load_csv_data(asaslog_file)


class AREAGeoFigureGenerator(GeoFigureGeneratorBase):
    """
    Generates the geographic figures showing the area conflicts
    and intrusions for a batch with given timestamp.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all geo figures will be
        saved is created in case it does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "geo", "area")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def load_conflict_location_data(self):
        """
        Load the locations of all conflicts from file.
        """

        summary_dir = os.path.join(self.batch_dir, "logfiles_summary")
        arealog_file = os.path.join(summary_dir, "arealog_locations.csv")
        self.location_list = load_csv_data(arealog_file)


class ComparisonFigureGeneratorBase(FigureGeneratorBase):
    """
    Abstract base class in which the shared functionalities of
    comparative plots are defined. The actual plot creation is to be
    implemented in a subclass for each different plot type.
    """

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.generate_all_figures()

    def generate_all_figures(self):
        """
        Generate all figures for each geometry in the batch.
        """

        for geometry in self.combination_dict:
            # Make figures based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "arealog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num conflicts",
                                    "area_conflicts")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num intrusions",
                                    "area_intrusions")

            # Make figures based on data in asaslog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            # self.make_single_figure(geometry,
            #                         df_geometry,
            #                         "duration [s]",
            #                         "conflict_duration")

            df_LoS_sev = df[(df["#geometry"] == geometry)
                            & (df["is LoS [-]"] == True)]
            # self.make_single_figure(geometry,
            #                         df_LoS_sev,
            #                         "duration [s]",
            #                         "los_duration")
            self.make_single_figure(geometry,
                                    df_LoS_sev,
                                    "LoS severity [-]",
                                    "los_severity")

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
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "route efficiency [-]",
                                    "efficiency")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "t in conf [%]",
                                    "t_in_conf")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "t in los [%]",
                                    "t_in_los")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "t in reso [%]",
                                    "t_in_reso")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num conf per ac [-]",
                                    "ac_conf")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num los per ac [-]",
                                    "ac_los")
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "num ac conf per/ac/dist [1/m]",
                                    "conf_per_dist")

            # Make figures based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
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

        try:
            # Use the min and max of the unfiltered column in the dataframe
            # to ensure all figures using this column have the same y-axis limits
            ymin = df[column].min()
            ymax = df[column].max()
            yrange = df[column].max() - df[column].min()

            # Set resolution method plotting orders
            reso_methods = ["OFF", "MVP", "EBY", "LFFT", "VEL_AVG",
                            "GV-METHOD1", "GV-METHOD2",
                            "GV-METHOD3", "GV-METHOD4"]
            reso_order = [method for method in reso_methods
                          if method in df["resolution method"].unique()]
            level_order = df["traffic level"].unique().sort()
            num_levels = df["traffic level"].nunique()

            # Make figure
            plt.figure(figsize=(16, 9))
            ax = self.create_plot(x="resolution method",
                                  y=column,
                                  data=df,
                                  order=reso_order,
                                  hue="traffic level",
                                  hue_order=level_order,
                                  linewidth=0.5,
                                #   showfliers=False,
                                  palette="Blues")
            plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 4))
            ax.legend(title="Traffic rate [aircraft/hr]",
                      loc="lower center",
                      ncol=num_levels,
                      bbox_to_anchor=(0.5, 1))
            ax.set(xticklabels=reso_order)
            yminplot = ymin - 0.05 * yrange
            ymaxplot = ymax + 0.05 * yrange
            ax.set_ylim(yminplot, ymaxplot)
            plt_filename = f"{geometry}_{namestr}.{FIGURE_FILETYPE}"
            plt_filepath = os.path.join(self.figure_dir, plt_filename)
            plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
            plt.close()
        except ValueError:
            print(f"Plot generator failed to create {plt_filename}")

    def create_plot(self, **kwargs):
        raise NotImplementedError


class BoxPlotFigureGenerator(ComparisonFigureGeneratorBase):
    """
    Generate the box plots for a scenario batch with given timestamp.
    """

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all box plot figures will
        be saved is created in case does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "boxplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def create_plot(self, **kwargs):
        """
        Returns a matplotlib Axes object with the box plot drawn in it
        """

        flierprops = dict(marker="+", markersize=5)
        return sbn.boxplot(**kwargs, flierprops=flierprops)


class ViolinPlotFigureGenerator(ComparisonFigureGeneratorBase):
    """
    Generate the violin plots for a scenario batch with given timestamp.
    """

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all violin plot figures will
        be saved is created in case does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "violinplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def create_plot(self, **kwargs):
        """
        Returns a matplotlib Axes object with the violin plot drawn in it
        """

        flierprops = dict(marker="+", markersize=5)
        return sbn.violinplot(**kwargs, flierprops=flierprops)


class StripPlotFigureGenerator(ComparisonFigureGeneratorBase):
    """
    Generate the strip plots for a scenario batch with given timestamp.
    """

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all strip plot figures will
        be saved is created in case does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "stripplot")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def create_plot(self, **kwargs):
        """
        Returns a matplotlib Axes object with the strip plot drawn in it
        """

        return sbn.stripplot(**kwargs, dodge=True, jitter=True)


class ASASConflictFigureGenerator(FigureGeneratorBase):

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.load_conflict_data()
        self.generate_figures()

    def create_figure_dir_if_not_exists(self):
        """
        Ensures that the subdirectory in which all geo figures will be
        saved is created in case it does not exist yet.
        """

        self.figure_dir = os.path.join(self.figure_dir, "conflict")
        if not os.path.isdir(self.figure_dir):
            os.makedirs(self.figure_dir)

    def load_conflict_data(self):
        """
        Load the locations of all conflicts from file.
        """

        summary_dir = os.path.join(self.batch_dir, "logfiles_summary")
        asaslog_file = os.path.join(summary_dir, "asaslog_occurence.csv")
        self.conflict_list = load_csv_data(asaslog_file)

    def generate_figures(self):
        """
        Creates the figures showing the geographic features of the scenario
        as well as the locations of the conflicts and losses of separation
        per separation method.
        """

        df = pd.read_csv(os.path.join(self.batch_dir,
                                      "logfiles_summary",
                                      "asaslog_occurence.csv"))
        for geometry in self.combination_dict:
            for method in self.combination_dict[geometry]:
                for level in self.combination_dict[geometry][method]:
                    # Make a figure
                    df_figure = df[(df["#geometry"] == geometry)
                                   & (df["resolution method"] == method)
                                   & (df["traffic level"] == int(level))
                                  ]
                    self.make_vel_angle_figure(df_figure,
                                               geometry,
                                               method,
                                               level)
                    # self.make_2D_boxplot_figure(df_figure,
                    #                             geometry,
                    #                             method,
                    #                             level)
                    self.make_dual_boxplot_figure(df_figure,
                                                  geometry,
                                                  method,
                                                  level)

        # Workaround for nested dictionary due to order reversal
        # of nesting required in iteration for this type of plot
        for geometry in self.combination_dict:
            histplot_dict = {}
            for method in self.combination_dict[geometry]:
                for level in self.combination_dict[geometry][method]:
                    if level not in histplot_dict:
                        histplot_dict[level] = []
                    histplot_dict[level].append(method)

            # Generate figure for each traffic level
            for level in histplot_dict.keys():
                plt.figure(figsize=(16, 9))
                x_axis = "delta trk [deg]"

                # Get data for current traffic level
                df_level = df[(df["#geometry"] == geometry)
                              & (df["is LoS [-]"] == False)
                              & (df["traffic level"] == int(level))]

                # Add data for each method to the figure
                for method in histplot_dict[level]:
                    # Get data for current separation method
                    df_method = df_level[(df_level["resolution method"] == method)]

                    bins = [x for x in range(181)]
                    sbn.distplot(df_method[x_axis],
                                 bins=bins,
                                # If use norm hist:
                                #  norm_hist=True,
                                #  hist=True,
                                #  kde=False,
                                #  label=method,
                                #  hist_kws={"histtype": "step",
                                #            "linewidth": 1,
                                #            "alpha": 1}
                                # If use kde:
                                 hist=False,
                                 kde=True,
                                 label=method,
                                 kde_kws={"kernel": "biw",
                                          "clip": (0, 180)},
                    )

                # Set attributes and save figure
                plt.xlabel(x_axis)
                plt.ylabel("Density")
                plt.axis("tight")
                plt.xlim((0, 180))
                plt.legend(title="Resolution method")
                plt_filename = f"hist_{geometry}_{level}.{FIGURE_FILETYPE}"
                plt_filepath = os.path.join(self.figure_dir, plt_filename)
                plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
                plt.close()

    def make_vel_angle_figure(self, df, geometry, method, level):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        try:
            # Set column labels to be used for data on x and y axes
            x_axis = "delta trk [deg]"
            y_axis = "delta v [kts]"

            # Create and save figure
            plt.figure(figsize=(16, 9))
            plt.scatter(x=x_axis,
                        y=y_axis,
                        data=df[df["is LoS [-]"] == False],
                        alpha=0.02,
                        edgecolors='none',
                        # marker="x",
                        )
            plt.xlabel(x_axis)
            plt.ylabel(y_axis)
            plt.axis([0, 180, 0, 1100])
            plt_filename = f"va_{geometry}_{method}_{level}.{FIGURE_FILETYPE}"
            plt_filepath = os.path.join(self.figure_dir, plt_filename)
            plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
            plt.close()
        except RuntimeError:
            print(f"Plot generator failed to create {plt_filename}")

    # def make_2D_boxplot_figure(self, df, geometry, method, level):
    #     """
    #     Make a single boxplot figure for a given geometry. The data used
    #     is specified in a pandas dataframe 'df', 'column' is the column used
    #     for the plot and 'namestr' is the last part of the figure file name.
    #     """

    #     try:
    #         # Set column labels to be used for data on x and y axes
    #         x_axis = "delta trk [deg]"
    #         y_axis = "delta v [kts]"

    #         # Create and save figure
    #         plt.figure(figsize=(16, 9))
    #         ax = plt.axes()

    #         df_plot = df[df["is LoS [-]"] == False]
    #         x = np.asarray(df_plot[x_axis])
    #         y = np.asarray(df_plot[y_axis])

    #         extra_plot.boxplot_2d(x, y, ax)
    #         plt.xlabel(x_axis)
    #         plt.ylabel(y_axis)
    #         plt.axis([0, 180, 0, 1100])
    #         plt_filename = f"2box_{geometry}_{method}_{level}.{FIGURE_FILETYPE}"
    #         plt_filepath = os.path.join(self.figure_dir, plt_filename)
    #         plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
    #         plt.close()
    #     except RuntimeError:
    #         print(f"Plot generator failed to create {plt_filename}")

    def make_dual_boxplot_figure(self, df, geometry, method, level):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        try:
            # Set column labels to be used for data on x and y axes
            x_axis = "delta trk [deg]"
            y_axis = "delta v [kts]"
            df_plot = df[df["is LoS [-]"] == False]

            # Create and save figure
            plt.figure()
            # plt.figure(figsize=(16, 9))
            _, (ax1, ax2) = plt.subplots(1, 2)
            sbn.boxplot(y=x_axis, data=df_plot, orient="v", ax=ax1)
            ax1.set_ylim((0, 180))
            # ax1.ylabel(x_axis)

            sbn.boxplot(y=y_axis, data=df_plot, orient="v", ax=ax2)
            ax2.set_ylim((0, 1100))
            # ax2.ylabel(y_axis)

            plt.tight_layout()
            plt_filename = f"dual_{geometry}_{method}_{level}.{FIGURE_FILETYPE}"
            plt_filepath = os.path.join(self.figure_dir, plt_filename)
            plt.savefig(plt_filepath, dpi=300, bbox_inches="tight")
            plt.close()
        except RuntimeError:
            print(f"Plot generator failed to create {plt_filename}")
