# Python imports
import os
import shutil

# Local imports
import output_parser


def process_batch(timestamp):
    """
    Process the BlueSky output files of the batch with given timestamp.

    The output files are copied to 'post_processing/<timestamp>/log'. All
    summarizing csv files are generated by parsing these copies.
    """

    # Set the source folder and create target folders if necessary
    source_dir = "output"
    batch_save_dir = os.path.join("post_processing", timestamp + "/")
    logfile_save_dir = os.path.join(batch_save_dir, "logfiles_raw/")
    summary_save_dir = os.path.join(batch_save_dir, "logfiles_summary/")

    for folder in [batch_save_dir, logfile_save_dir, summary_save_dir]:
        if not os.path.isdir(folder):
            os.makedirs(folder)

    # Copy the BlueSky logfiles that are part of the batch to the folder
    # /post_processing/<timestamp>/log
    source_files = [filename for filename in os.listdir(source_dir)
                    if timestamp in filename]
    for logfile in source_files:
        shutil.copy2(os.path.join(source_dir, logfile),
                     os.path.join(logfile_save_dir, logfile))
    logfiles = [filename for filename in os.listdir(logfile_save_dir)]

    # Process the ASAS log files
    asaslogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "ASASLOG" in filename]
    output_parser.ASASLogSummaryParser(
        asaslogfiles,
        os.path.join(summary_save_dir, "asaslog_summary.csv")
    )
    output_parser.ASASLogOccurrenceParser(
        asaslogfiles,
        os.path.join(summary_save_dir, "asaslog_occurence.csv")
    )
    output_parser.ASASLogLocationParser(
        asaslogfiles,
        os.path.join(summary_save_dir, "asaslog_locations.csv")
    )

    # Process the AREA log files
    arealogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "AREALOG" in filename]
    output_parser.AREALogSummaryParser(
        arealogfiles,
        os.path.join(summary_save_dir, "arealog_summary.csv")
    )

    # Process the FLST log files
    flstlogfiles = [os.path.join(logfile_save_dir, filename)
                    for filename in logfiles if "FLSTLOG" in filename]
    output_parser.FLSTLogOccurrenceParser(
        flstlogfiles,
        os.path.join(summary_save_dir, "flstlog_occurence.csv")
    )


if __name__ == "__main__":
    timestamp = "20190701-034019"
    process_batch(timestamp)
