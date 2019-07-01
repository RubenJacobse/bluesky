"""
Generate figures showing conflict locations.
"""

# Python imports
import os
import sys
import csv
from copy import copy

# Third-party imports
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

# Enable BlueSky imports by adding the project folder to the path
sys.path.append(os.path.abspath(os.path.join('..')))

# BlueSky imports
import bluesky.tools.geo as bsgeo


def main():
    """
    Bla
    """

    timestamp = "20190701-034019"
    combi_file_name = os.path.join("scenario",
                                   timestamp,
                                   "combinations.csv")

    with open(combi_file_name, "r") as combi_file:
        combi_dict = {}
        for combination in csv.reader(combi_file, delimiter=","):
            # Unpack the current row, except if it commented out
            if combination[0].startswith("#"):
                continue
            [scenname, length, width, angle, method] = combination

            # Add combination to dictionary
            geometry = f"L{length}_W{width}_A{angle}"
            if not geometry in combi_dict.keys():
                combi_dict[geometry] = {}
            if not method in combi_dict[geometry].keys():
                combi_dict[geometry][method] = []
            combi_dict[geometry][method].append(scenname)

    with open(f"post_processing\{timestamp}\\asaslog_locations.csv") as asaslog_file:
        asaslog_reader = csv.reader(asaslog_file, delimiter=",")
        asaslog_data = [row for row in list(asaslog_reader)
                        if not row[0].startswith("#")]

    for geometry in combi_dict:
        # Create and save the bare plot with geometry only
        geo_file = f"scenario/{timestamp}/{timestamp}_{geometry}_geo.csv"
        geo_data = load_geo_data(geo_file)
        geo_plot = plot_geo_data(geo_data)
        geo_plot.savefig(f"L{length}_W{width}_A{angle}.png", dpi=300)
        geo_plot.close()

        # Create plots showing conflict locations for each resolution method
        for method in combi_dict[geometry]:
            conflict_locations = [[], []]
            for row in asaslog_data:
                if geometry in row[0] and method in row[0]:
                    conflict_locations[0].append(float(row[2]))
                    conflict_locations[1].append(float(row[1]))

            conf_plot = plot_geo_data(geo_data)
            conf_plot.scatter(conflict_locations[0],
                              conflict_locations[1],
                              alpha=0.1,
                              linewidths=None)
            conf_plot.title(f"Separation method: {method}")
            conf_plot.savefig(f"L{length}_W{width}_A{angle}_{method}.png", dpi=300)


def load_geo_data(geo_file):
    """
    Load the geometry data from the csv format in 'geo_file' to a list
    that contains lists of row elements in string format.
    """

    with open(geo_file) as geo_file:
        geo_reader = csv.reader(geo_file, delimiter=",")
        data = [row for row in list(geo_reader)
                if not row[0].startswith("#")]

    return data


def plot_geo_data(geo_data):
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

    # Generate polygon for area on left side of corridor
    larea_name = geo_data[1][0]
    larea_coords = [(float(lon), float(lat)) for (lat, lon)
                    in zip(geo_data[1][1:8:2], geo_data[1][2:9:2])]
    larea_polygon = Polygon(larea_coords)

    # Generate polygon for area on right side of corridor
    rarea_name = geo_data[1][0]
    rarea_coords = [(float(lon), float(lat)) for (lat, lon)
                    in zip(geo_data[2][1:8:2], geo_data[2][2:9:2])]
    rarea_polygon = Polygon(rarea_coords)

    # Calculate the intersections of the area polygons and the ring polygon
    larea_inring = ring_polygon.intersection(larea_polygon)
    rarea_inring = ring_polygon.intersection(rarea_polygon)

    # Create the actual plot
    plt.figure()
    plt.fill(*larea_inring.exterior.xy,
             facecolor="xkcd:pale pink",
             edgecolor="xkcd:brick red",
             linewidth=1)
    plt.fill(*rarea_inring.exterior.xy,
             facecolor="xkcd:pale pink",
             edgecolor="xkcd:brick red",
             linewidth=1)
    plt.plot(*ring_polygon.exterior.xy, "k", linewidth=1)
    plt.axis("scaled")
    plt.xlabel("longitude [deg]")
    plt.ylabel("latitude [deg]")

    return plt


def make_ac_conflict_location_plot():
    conf_coords_lat = []
    conf_coords_lon = []

    with open("asaslog_locations.csv") as asasloc_file:
        locfile_reader = csv.reader(asasloc_file, delimiter=",")
        for row in locfile_reader:
            # Only process rows
            if not reso_method in row[0]:
                continue

    # Plot coordinates of conflicts
    plt.figure(1)
    plt.title("Hoi")
    plt.plot(conf_coords_lon, conf_coords_lat, "ro")


def make_ac_los_location_plot():
    pass


if __name__ == "__main__":
    main()
