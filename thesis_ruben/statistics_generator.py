"""
Generate tables with statistical data.
"""

# Python imports
import csv
import os
import itertools

# Third-party imports
from matplotlib import pyplot
from scipy import stats
import pandas as pd
import numpy as np

# Module level constants
SIGNIFICANCE_LEVEL = 0.05
TEX_HEADER_STR = \
    ("\\documentclass[a4paper]{report}\n\\usepackage{booktabs}\n"
     + "\\usepackage{fullpage}\\begin{document}\n\n\\section{}")
TEX_FOOTER_STR = "\n\\end{document}"


def make_batch_statistics(timestamp):
    """
    Generate the tables for the batch with given timestamp.
    """

    # Generate tables with normality statistics
    print("\nKolmogorov:")
    KolmogorovTableGenerator(timestamp)
    print("\nShapiro:")
    ShapiroTableGenerator(timestamp)

    # Generate tables with comparative statistics
    print("\nWilcoxon")
    WilcoxonTableGenerator(timestamp)
    print("\nMannWhitney")
    MannWhitneyTableGenerator(timestamp)


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


def flt_to_latex(flt):
    """ Convert floating point number to latex text format. """

    if str(flt).upper() == "NAN":
        latex_str = flt
    else:
        latex_str = "{0}e{{{1}}}".format(*f"{flt:.1E}".split("E"))
    return latex_str


def abbrev(method):
    """ Abbreviate strings with format GV-METHOD<x> to GV-M<x>"""
    if "GV-METHOD" in method:
        abb = "GV-M" + method[-1]
    else:
        abb = method
    return abb


def create_dir_if_not_exists(directory: str) -> None:
    """ Creates the directory if it does not exist yet. """

    if not os.path.isdir(directory):
        os.makedirs(directory)


class TableGeneratorBase:
    """
    Base class that defines the directory in which all tables will be saved
    for a given batch. Also creates this directory if it does not exist yet.
    """

    def __init__(self, timestamp):
        self.timestamp = timestamp
        self.combination_dict = {}
        self.tex_table_list = []

        self.batch_dir = os.path.join("post_processing", self.timestamp)
        self.table_dir = os.path.join(self.batch_dir, "tables")
        create_dir_if_not_exists(self.table_dir)
        self.make_combination_dict()

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

    def generate_all_tables(self):
        """
        Generate all tables for each geometry in the batch.
        """

        for geometry in self.combination_dict:
            # Make tables based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "arealog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num conflicts",
                                   "area_conflicts")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num intrusions",
                                   "area_intrusions")

            # Make tables based on data in asaslog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            print("Parsing asaslog_occurence.csv")
            # self.make_single_table(geometry,
            #                         df_geometry,
            #                         "duration [s]",
            #                         "conflict_duration")

            df_los = df[(df["#geometry"] == geometry)
                        & (df["is LoS [-]"] == True)]
            # self.make_single_table(geometry,
            #                         df_los,
            #                         "duration [s]",
            #                         "los_duration")
            self.make_single_table(geometry,
                                   df_los,
                                   "LoS severity [-]",
                                   "los_severity")
            self.make_single_table(geometry,
                                   df_los,
                                   "delta v [kts]",
                                   "los_delta_v")
            self.make_single_table(geometry,
                                   df_los,
                                   "delta trk [deg]",
                                   "los_delta_trk")

            # Make tables based on data in asaslog_summary.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "asaslog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            print("Parsing asaslog_summary.csv")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num conflicts [-]",
                                   "num_conflicts")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num LoS [-]",
                                   "num_LoS")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "IPR [-]",
                                   "IPR")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "DEP [-]",
                                   "DEP")

            # Make tables based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_occurence.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            print("Parsing flstlog_occurence.csv")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "route efficiency [-]",
                                   "efficiency")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "t in conf [%]",
                                   "t_in_conf")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "t in los [%]",
                                   "t_in_los")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "t in reso [%]",
                                   "t_in_reso")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num conf per ac [-]",
                                   "ac_conf")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num los per ac [-]",
                                   "ac_los")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num ac conf per/ac/dist [1/m]",
                                   "conf_per_dist")

            # Make tables based on data in flstlog_occurence.csv
            df = pd.read_csv(os.path.join(self.batch_dir,
                                          "logfiles_summary",
                                          "flstlog_summary.csv"))
            df_geometry = df[df["#geometry"] == geometry]
            print("Parsing flstlog_summary.csv")
            self.make_single_table(geometry,
                                   df_geometry,
                                   "num turnaround [-]",
                                   "num_turnaround")

    def make_single_table(self, *args):
        raise NotImplementedError


class NormalityTableGeneratorBase(TableGeneratorBase):

    def make_single_table(self, geometry, df, column, namestr):
        """
        Make a normality statistics table for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the table file name.
        """

        table_filename = f"{geometry}_{namestr}"
        table_csvpath = os.path.join(self.table_subdir, table_filename + ".csv")
        table_texpath = os.path.join(self.table_subdir, table_filename + ".tex")

        try:
            # Set table column and row orders
            reso_methods = ["OFF", "MVP", "EBY", "VELAVG",
                            "GV-METHOD1", "GV-METHOD2",
                            "GV-METHOD3", "GV-METHOD4"]
            reso_order = [method for method in reso_methods
                          if method in df["resolution method"].unique()]
            level_order = df["traffic level"].unique()
            level_order.sort()
            num_levels = df["traffic level"].nunique()

            # Set up empty dataframe for table
            table_df = pd.DataFrame(index=level_order, columns=reso_order)

            # For each method-level combination add p-value to dataframe
            combinations = list(itertools.product(reso_order, level_order))
            for (method, level) in combinations:
                df_combination = df[(df["traffic level"] == level)
                                    & (df["resolution method"] == method)]
                data = df_combination[column]

                try:
                    test_stat, p_val = self.stat_test(data)
                    p_val_str = flt_to_latex(p_val)
                    if p_val < SIGNIFICANCE_LEVEL:
                        p_val_str = f"\\underline{{{p_val_str}}}"
                    table_df.at[level, method] = p_val_str
                except ValueError:
                    print(f"\tValueError!")
                    print(f"\t{column}, {method}, {level}")
                    table_df.at[level, method] = "NaN"

            caption = f"P-values for {self.test_name} tests for {namestr.replace('_', ' ')}"
            # table_df.reset_index().to_csv(table_csvpath, index=False,
            #                               header=True, decimal='.', sep=',',
            #                               float_format="%.3E")
            table_df.to_latex(table_texpath, escape=False, caption=caption)
            latex_str = table_df.to_latex(buf=None, escape=False, caption=caption,
            column_format="c" * (len(reso_order) + 1))
            self.tex_table_list.append(latex_str)
        except Exception as e:
            print(f"Table generator failed to create {table_filename}")
            raise e

    @staticmethod
    def stat_test(*args):
        raise NotImplementedError


class ComparisonTableGeneratorBase(TableGeneratorBase):

    def make_single_table(self, geometry, df, column, namestr):
        """
        Make a comparative statictics table for a given geometry. The data used
        is specified in a pandas dataframe 'df', 'column' is the column used
        for the plot and 'namestr' is the last part of the table file name.
        """

        table_filename = f"{geometry}_{namestr}"
        table_csvpath = os.path.join(self.table_subdir, table_filename + ".csv")
        table_texpath = os.path.join(self.table_subdir, table_filename + ".tex")

        try:
            # Set resolution method plotting orders
            reso_methods = ["EBY", "VELAVG",
                            "GV-METHOD1", "GV-METHOD2",
                            "GV-METHOD3", "GV-METHOD4"]
            reso_order = [method for method in reso_methods
                          if method in df["resolution method"].unique()]
            level_order = df["traffic level"].unique()
            level_order.sort()
            num_levels = df["traffic level"].nunique()

            # Set up empty dataframe for table
            pval_df = pd.DataFrame(index=level_order, columns=reso_order)
            stat_df = pd.DataFrame(index=level_order, columns=reso_order)

            # For each method-level combination add p-value to dataframe
            combinations = list(itertools.product(reso_order, level_order))
            for (method, level) in combinations:
                df_baseline = df[(df["traffic level"] == level)
                                 & (df["resolution method"] == "MVP")]
                df_combination = df[(df["traffic level"] == level)
                                    & (df["resolution method"] == method)]
                base_data = df_baseline[column]
                data = df_combination[column]

                try:
                    test_stat, p_val = self.stat_test(base_data, data)
                    p_val_str = flt_to_latex(p_val)
                    test_stat_str = flt_to_latex(test_stat)
                    if p_val < SIGNIFICANCE_LEVEL:
                        p_val_str = f"\\underline{{{p_val_str}}}"
                    pval_df.at[level, method] = p_val_str
                    stat_df.at[level, method] = test_stat_str
                except ValueError as e:
                    print(f"\tValueError!")
                    print(f"\t{column}, {method}, {level}")
                    pval_df.at[level, method] = "NaN"
                    stat_df.at[level, method] = "NaN"

            caption = ("Test statistic (top) and p-values (bottom) for "
                       + f"{self.test_name} tests for {namestr.replace('_', ' ')}")
            table_df = pd.concat([stat_df, pval_df])
            # table_df.reset_index().to_csv(table_csvpath, index=False,
            #                               header=True, decimal='.', sep=',',
            #                               float_format="%.3E")
            latex_str = table_df.to_latex(buf=None, escape=False, caption=caption,
                                          column_format="c" * (len(reso_order) + 1))
            self.tex_table_list.append(latex_str)
        except Exception as e:
            print(f"Table generator failed to create {table_filename}")
            raise e

    @staticmethod
    def stat_test(*args):
        raise NotImplementedError


class KolmogorovTableGenerator(NormalityTableGeneratorBase):

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.test_name = "kolmogorov"
        self.table_subdir = os.path.join(self.table_dir, "norm_" + self.test_name)
        create_dir_if_not_exists(self.table_subdir)
        tex_summary_file = os.path.join(self.table_subdir, "summary.tex")

        with open(tex_summary_file, 'w') as tex_summary:
            self.generate_all_tables()

            tex_summary.write(TEX_HEADER_STR)
            for tex_table in self.tex_table_list:
                tex_summary.write("\n" + tex_table)
            tex_summary.write(TEX_FOOTER_STR)

    @staticmethod
    def stat_test(x):
        return stats.kstest(x, "norm")


class ShapiroTableGenerator(NormalityTableGeneratorBase):

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.test_name = "shapiro"
        self.table_subdir = os.path.join(self.table_dir, "norm_" + self.test_name)
        create_dir_if_not_exists(self.table_subdir)
        tex_summary_file = os.path.join(self.table_subdir, "summary.tex")

        with open(tex_summary_file, 'w') as tex_summary:
            self.generate_all_tables()

            tex_summary.write(TEX_HEADER_STR)
            for tex_table in self.tex_table_list:
                tex_summary.write("\n" + tex_table)
            tex_summary.write(TEX_FOOTER_STR)

    @staticmethod
    def stat_test(x):
        return stats.shapiro(x)


class WilcoxonTableGenerator(ComparisonTableGeneratorBase):

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.test_name = "wilcoxon"
        self.table_subdir = os.path.join(self.table_dir, "comp_" + self.test_name)
        create_dir_if_not_exists(self.table_subdir)
        tex_summary_file = os.path.join(self.table_subdir, "summary.tex")

        with open(tex_summary_file, 'w') as tex_summary:
            self.generate_all_tables()

            tex_summary.write(TEX_HEADER_STR)
            for tex_table in self.tex_table_list:
                tex_summary.write("\n" + tex_table)
            tex_summary.write(TEX_FOOTER_STR)

    @staticmethod
    def stat_test(x, y):
        return stats.wilcoxon(x, y)


class MannWhitneyTableGenerator(ComparisonTableGeneratorBase):

    def __init__(self, timestamp):
        super().__init__(timestamp)
        self.test_name = "mannwhitney"
        self.table_subdir = os.path.join(self.table_dir, "comp_" + self.test_name)
        create_dir_if_not_exists(self.table_subdir)
        tex_summary_file = os.path.join(self.table_subdir, "summary.tex")

        with open(tex_summary_file, 'w') as tex_summary:
            self.generate_all_tables()

            tex_summary.write(TEX_HEADER_STR)
            for tex_table in self.tex_table_list:
                tex_summary.write("\n" + tex_table)
            tex_summary.write(TEX_FOOTER_STR)

    @staticmethod
    def stat_test(x, y):
        return stats.mannwhitneyu(x, y)
