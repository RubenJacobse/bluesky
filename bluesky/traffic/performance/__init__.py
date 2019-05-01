""" Performance package imports """

# Register default configuration variable
from bluesky import settings
settings.set_variable_defaults(performance_model="openap")

# Import performance model based on settings
if settings.performance_model == "bada":
    try:
        from .bada.perfbada import PerfBADA as Perf
        print("Using BADA Performance model")
    except ImportError:
        settings.performance_model = "openap"
        print("Error: BADA performance model not available, falling back" \
              + "to Open Aircraft Performance (OpenAP) model")
        from .openap import OpenAP as Perf
elif settings.performance_model == "openap":
    from .openap import OpenAP as Perf
    print("Using Open Aircraft Performance (OpenAP) model")
else:
    from .legacy.perfbs import PerfBS as Perf
    print("Using BlueSky legacy performance model")
