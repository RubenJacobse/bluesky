""" Tools package imports """

# Make available in the BlueSky package structure
from . import cachefile
from .walltime import Timer
from .trafficarrays import RegisterElementParameters, TrafficArrays
from .signal import Signal

# Register default configuration variable
from bluesky import settings
settings.set_variable_defaults(prefer_compiled=False)

# Import geo calculation module. Use the compiled version if preferred,
# otherwise revert back to the Python version. This is hidden from the caller
# which imports the selected module using 'from bluesky.tools import geo'.
geo = None
if settings.prefer_compiled:
    try:
        from .src_cpp import cgeo as geo
        print("Using compiled geo functions.")
    except ImportError:
        print("No compiled geo functions for this platform.")

if not geo:
    from . import geo
    print("Using Python-based geo functions")
