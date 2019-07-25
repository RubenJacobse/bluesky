"""
Generate BlueSky scenario files using scenario_file_generator.py for specified
combinations of input variables.
"""

# Python imports
import os
import shutil
import itertools
import datetime

# Local imports
from scenario_file_generator import ScenarioGenerator


def main():
    """
    Use the scenario generator to create a number of scenario files
    using all possible combinations of the relevant input parameter
    lists. Also create a scenario file that allows the BlueSky
    BATCH command to be used to execute the generated scenarios in
    parallel on all cpu cores.
    """

    # Set directory where scenarios and other files are stored (create
    # new folder if not already existing)
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    current_dir = os.path.dirname(__file__)
    scen_dir = f"{timestamp}" + os.sep
    output_dir = os.path.join(current_dir, "scenario", scen_dir)
    if not os.path.exists(os.path.dirname(output_dir)):
        os.mkdir(os.path.dirname(output_dir))

    # Input variables and the list of all combinations
    random_seed = [x for x in range(1, 2)]
    traffic_level = ["LOW", "MID", "HIGH"]
    asas_reso_method = ["OFF"]
    corridor_length = [40]
    corridor_width = [25]
    ac_creation_arc_angle = [100]
    combination_lst = list(itertools.product(random_seed,
                                             traffic_level,
                                             asas_reso_method,
                                             corridor_length,
                                             corridor_width,
                                             ac_creation_arc_angle))

    # Create a scenario file for each combination of variables
    for (seed, level, reso_method, length, width, angle) in combination_lst:
        ScenarioGenerator(output_dir,
                        timestamp,
                        seed,
                        level,
                        reso_method,
                        length,
                        width,
                        angle,
                        is_edge_angle=False)

    # Create a batch file that allows the BlueSky "BATCH" command to
    # process all scenario files using all CPU cores in parallel.
    with open(os.path.join(output_dir, "batch.scn"), "w") as batch_file:
        for (seed, level, reso_method, length, width, angle) in combination_lst:
            scenfile_name = ("{}_L{}_W{}_A{}_RESO-{}_T-{}_SCEN{:03d}.scn"
                             .format(timestamp,
                                     length,
                                     width,
                                     angle,
                                     reso_method,
                                     level,
                                     seed))
            scenfile_path = os.path.join(scen_dir, scenfile_name)
            batch_file.write("0:00:00.00>SCEN {}\n".format(scenfile_name[:-4]))
            batch_file.write("0:00:00.00>PCALL {}\n".format(scenfile_path))
            batch_file.write("0:00:00.00>SCHEDULE 5:00:00 HOLD\n")

        # Exit BlueSky after last scenario is finished
        batch_file.write("0:00:00.00>SCHEDULE 5:00:00 EXIT")

    # Make a copy of the batch file to simplify calling from the
    # BlueSky command line by allowing the use of "BATCH thesis_latest"
    shutil.copy(os.path.join(output_dir, "batch.scn"),
                os.path.join(current_dir, "scenario", "thesis_latest.scn"))

    # Store all combinations in csv format for easier post-processing
    with open(os.path.join(output_dir, "combinations.csv"), "w") as combi_file:
        combi_header = ("# scen name, corridor length, corridor width, angle,"
                        + " reso method, traffic level\n")
        combi_file.write(combi_header)
        for combination in combination_lst:
            (seed, level, reso_method, length, width, angle) = combination
            scen_name = "SCEN{:03d}".format(seed)
            combi_file.write(f"{scen_name},{length},{width},{angle},"
                             + f"{reso_method},{level}\n")


if __name__ == "__main__":
    main()
