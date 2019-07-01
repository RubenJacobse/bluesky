# Python imports
import os
import shutil

# Local imports
import output_parser


def process_batch(timestamp):
    """
    Process the output files of the batch with given timestamp.
    """

    logfile_dir = "output"
    processing_dir = os.path.join("post_processing", timestamp)
    if not os.path.isdir(processing_dir):
        os.makedirs(processing_dir + "/")
        os.makedirs(os.path.join(processing_dir, "log/"))

    # Generate the lists of logfiles that are part of the batch and copy to
    # folder /post_processing/<timestamp>
    logfiles = [logfile for logfile in os.listdir(logfile_dir)
                if timestamp in logfile]
    for logfile in logfiles:
        shutil.copy2(os.path.join(logfile_dir, logfile),
                     os.path.join(processing_dir, "log", logfile))

    # Process the ASAS log files
    asaslogfiles = [os.path.join(processing_dir, "log", logfile)
                    for logfile in logfiles if "ASASLOG" in logfile]
    output_parser.ASASLogSummaryParser(
        asaslogfiles,
        os.path.join(processing_dir, "asaslog_summary.csv"),
        "#logfile, num conflicts [-], num LoS [-], IPR [-]"
    )
    output_parser.ASASLogOccurrenceParser(
        asaslogfiles,
        os.path.join(processing_dir, "asaslog_occurence.csv"),
        "#logfile, confpair, conflict duration [s], is LoS [-], LoS severity [-], t start [s], t end[s]"
    )
    output_parser.ASASLogLocationParser(
        asaslogfiles,
        os.path.join(processing_dir, "asaslog_locations.csv"),
        "#logfile, ac lat [deg], ac lon[deg], is LoS [-]"
    )

    # Process the AREA log files
    arealogfiles = [os.path.join(processing_dir, "log", logfile)
                    for logfile in logfiles if "AREALOG" in logfile]
    output_parser.AREALogSummaryParser(
        arealogfiles,
        os.path.join(processing_dir, "arealog_summary.csv"),
        "#logfile, num intrusions [-]"
    )

    # Process the FLST log files
    flstlogfiles = [os.path.join(processing_dir, "log", logfile)
                    for logfile in logfiles if "FLSTLOG" in logfile]
    output_parser.FLSTLogOccurrenceParser(
        flstlogfiles,
        os.path.join(processing_dir, "flstlog_occurence.csv"),
        "#logfile, ac id, work [J], route efficiency [-], dist to last wp [m]"
    )


if __name__ == "__main__":
    timestamp = "20190701-034019"
    process_batch(timestamp)
