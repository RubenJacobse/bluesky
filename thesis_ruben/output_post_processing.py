# Python imports
import os
import shutil

# Local imports
import output_parser
from plot_generator import make_batch_figures
from statistics_generator import make_batch_statistics


def summarize_logfiles(timestamp):
    """
    Process the BlueSky output files of the batch with given timestamp.

    The output files are copied to 'post_processing/<timestamp>/logfiles_raw/'.
    All summarizing csv files are generated by parsing these copies and are
    stored in 'post_processing/<timestamp>/logfiles_summary/'.
    """

    # Set the source folder and create target folders if necessary
    log_source_dir = "output"
    geom_source_dir = os.path.join("scenario", timestamp + os.sep)
    batch_save_dir = os.path.join("post_processing", timestamp + os.sep)
    logfile_save_dir = os.path.join(batch_save_dir, "logfiles_raw" + os.sep)
    geomfile_save_dir = os.path.join(batch_save_dir, "geomfiles" + os.sep)
    summary_save_dir = os.path.join(batch_save_dir, "logfiles_summary" + os.sep)

    for folder in [batch_save_dir, logfile_save_dir,
                   summary_save_dir, geomfile_save_dir]:
        if not os.path.isdir(folder):
            os.makedirs(folder)

    # Copy the BlueSky logfiles that are part of the batch to the folder
    # /post_processing/<timestamp>/logfiles_raw
    source_files = [filename for filename in os.listdir(log_source_dir)
                    if timestamp in filename]
    for logfile in source_files:
        shutil.copy2(os.path.join(log_source_dir, logfile),
                     os.path.join(logfile_save_dir, logfile))
    logfiles = [filename for filename in os.listdir(logfile_save_dir)]

    # Copy the geometry files that are part of the batch to the folder
    # /post_processing/<timestamp>/geomfiles
    geom_files = [filename for filename in os.listdir(geom_source_dir)
                  if timestamp in filename and filename.endswith(".csv")]
    for geomfile in geom_files:
        shutil.copy2(os.path.join(geom_source_dir, geomfile),
                     os.path.join(geomfile_save_dir, geomfile))

    # Process the AREA log files
    arealogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "AREALOG" in filename]
    arealogfiles.sort()
    output_parser.AREALogSummaryParser(
        arealogfiles,
        os.path.join(summary_save_dir, "arealog_summary.csv")
    )
    output_parser.AREALogLocationParser(
        arealogfiles,
        os.path.join(summary_save_dir, "arealog_locations.csv")
    )

    # Process the FLST log files
    flstlogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "FLSTLOG" in filename]
    flstlogfiles.sort()
    output_parser.FLSTLogOccurrenceParser(
        flstlogfiles,
        os.path.join(summary_save_dir, "flstlog_occurence.csv")
    )
    output_parser.FLSTLogSummaryParser(
        flstlogfiles,
        os.path.join(summary_save_dir, "flstlog_summary.csv")
    )

    # Process the ASAS log files
    asaslogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "ASASLOG" in filename]
    asaslogfiles.sort()
    output_parser.ASASLogSummaryParser(
        asaslogfiles,
        os.path.join(summary_save_dir, "asaslog_summary.csv")
    )
    output_parser.ASASLogOccurrenceParser(
        asaslogfiles,
        os.path.join(summary_save_dir, "asaslog_occurence.csv")
    )

    # Process the ASAS pos files
    asasposfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "ASASPOS" in filename]
    asasposfiles.sort()
    output_parser.ASASPosLocationParser(
        asasposfiles,
        os.path.join(summary_save_dir, "asaslog_locations.csv")
    )

    print("Finished summarizing logfiles")

if __name__ == "__main__":
    timestamp = "20200221-134518"
    summarize_logfiles(timestamp)
    make_batch_figures(timestamp)
    make_batch_statistics(timestamp)
