""" ASAS package imports """

# Make ASAS class available in the BlueSky package structure
from .asas import ASAS

# Register default configuration variable
from bluesky import settings
settings.set_variable_defaults(prefer_compiled=False)

# Import conflict detection module. Use the compiled version if preferred,
# otherwise revert back to the Python version. This is hidden from the ASAS
# class which imports the selected module using 'from . import StateBasedCD'.
StateBasedCD = None
if settings.prefer_compiled:
    try:
        from .src_cpp import casas as StateBasedCD
        print("StateBasedCD: using compiled version.")
    except ImportError:
        print("StateBasedCD: no compiled version for this platform.")

if not StateBasedCD:
    print("StateBasedCD: using Python version.")
    from . import StateBasedCD
