"""
Quick and dirty script to generate zoomed IPR boxplot
"""

# Python imports
import os
import sys
import csv
import itertools

# Third-party imports
import numpy as np
import pandas as pd
import seaborn as sbn
import matplotlib.pyplot as plt
from scipy import optimize
from matplotlib import axes
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from shapely.geometry import Polygon
from sklearn.metrics import mean_squared_error
from sklearn.model_selection import train_test_split
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join("..")))

# BlueSky imports
import bluesky.tools.geo as bsgeo

# Geo plot element colors
RESTRICTION_FACECOLOR = "xkcd:pale pink"
RESTRICTION_EDGECOLOR = "xkcd:brick red"
GEOVECTOR_EDGECOLOR = "xkcd:boring green"
GEOVECTOR_FACECOLOR = "xkcd:light seafoam green"
SWARMZONE_EDGECOLOR = "xkcd:sand"
SWARMZONE_FACECOLOR = "xkcd:light tan"
INTRUSION_MARKER_COLOR = "red"
CONFLICT_MARKER_COLOR = "blue"

# Save figures as following type
FIGURE_FILETYPE = "pdf"
FIGURE_SIZE = (8, 4.5)

# Various
PARAM_GUESS_INIT = [1000, 100] # Initial guess for CAMDA ['rho_max', 'k']
ASAS_FIGURE_SCENARIOS = ["031", "088"] # Scenarios used in ASAS location figures


def make_batch_figures(timestamp):
    """
    Generate the figures for the batch with given timestamp.
    """

    print("Generating boxplot figures")
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


def iterload_csv_data(filename, delimiter=",", comment_token="#"):
    """
    Loads the csv data from filename and returns a list that contains lists
    of row elements in string format.
    """

    with open(filename) as csv_file:
        reader = csv.reader(csv_file, delimiter=delimiter)
        data = [row for row in reader
                if (not row[0].startswith(comment_token)
                    and row[3] in ASAS_FIGURE_SCENARIOS)]
    return data


def create_dir_if_not_exists(directory: str) -> None:
    """ Creates the directory if it does not exist yet. """

    if not os.path.isdir(directory):
        os.makedirs(directory)


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
                                          "asaslog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_figure(geometry,
                                    df_geometry,
                                    "IPR [-]",
                                    "IPR",
                                    r"$IPR$ [-]")


    def make_single_figure(self, geometry, df, column, namestr,
                           varstr, showfliers=False, showbase=False):
        """
        Make a single boxplot figure for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the figure file name.
        """

        # If not showbase, do not include resolution method "OFF"
        if not showbase:
            df = df[(df["resolution method"] != "OFF")]

        # # Use the min and max of the unfiltered column in the dataframe
        # # to ensure all figures using this column have the same y-axis limits
        # ymin = df[column].min()
        # ymax = df[column].max()
        # yrange = df[column].max() - df[column].min()

        # Set resolution method plotting orders
        base_method = []
        reso_methods = base_method + ["MVP", "VELAVG",
                                      "GV-METHOD1", "GV-METHOD2",
                                      "GV-METHOD3", "GV-METHOD4"] # removed MVP
        reso_order = [method for method in reso_methods
                      if method in df["resolution method"].unique()]
        reso_label = base_method + ["MVP", "VELAVG", "GV-SPD", "GV-ZONES",
                                    "GV-RINGS", "GV-GRID"]
        level_order = df["traffic level"].unique().sort()
        num_levels = df["traffic level"].nunique()

        # Make figure
        plt.figure(figsize=FIGURE_SIZE)
        plt.rc('text', usetex=True)
        plt.rc('font', size=12)
        ax = self.create_plot(x="resolution method",
                              y=column,
                              data=df,
                              order=reso_order,
                              hue="traffic level",
                              hue_order=level_order,
                              linewidth=0.5,
                              showfliers=showfliers,
                              palette="Blues")
        plt.ticklabel_format(axis="y", style="sci", scilimits=(0, 4))
        ax.legend(title="Traffic rate [aircraft/hr]",
                  loc="lower center",
                  ncol=num_levels,
                  bbox_to_anchor=(0.5, 1))
        ax.set(xticklabels=reso_label)
        plt.xlabel("Separation method")
        plt.ylabel(varstr)
        # plt.setp(ax.get_xticklabels(), rotation=30, horizontalalignment='right')
        # Next three lines are useful to ensure the same plot types have
        # the same axes when comparing different geometries
        # yminplot = ymin - 0.05 * yrange
        # ymaxplot = ymax + 0.05 * yrange
        # ax.set_ylim(yminplot, ymaxplot)

        # Add inset in new axis
        axins = inset_axes(ax, "70%", "58%", loc="lower center", borderpad=0.7)
        sbn.boxplot(x="resolution method",
                    y=column,
                    data=df,
                    order=reso_order,
                    hue="traffic level",
                    hue_order=level_order,
                    linewidth=0.5,
                    showfliers=showfliers,
                    palette="Blues",
                    ax=axins)
        axins.get_legend().remove()
        axins.axes.xaxis.set_visible(False)
        axins.axes.yaxis.label.set_visible(False)
        axins.axes.set_yticks([0.99, 1.00])
        axins.set_xlim(0.5, 5.5)
        axins.set_ylim(0.9895, 1.0005)
        mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec='0.5', lw='0.5')

        plt_filename = f"{geometry}_{namestr}_zoomed.{FIGURE_FILETYPE}"
        plt_filepath = os.path.join(self.figure_dir, plt_filename)
        try:
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


if __name__ == "__main__":
    make_batch_figures("20200221-134518")
