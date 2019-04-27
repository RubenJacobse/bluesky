#!/usr/bin/env python
""" Main BlueSky start script """

import sys
import traceback
import bluesky as bs

def exception_handler(exc_type, exc_value, exc_traceback):
    """
    Create custom system-wide exception handler. For now it replicates Python's
    default traceback message. This was added to counter a new PyQt5.5 feature
    where unhandled exceptions would result in a qFatal with a very uninformative
    message.
    """

    traceback.print_exception(exc_type, exc_value, exc_traceback)
    sys.exit()


sys.excepthook = exception_handler


def main():
    """
    Start BlueSky: This is the main entry point for BlueSky. Depending on
    settings and arguments passed it can start in different modes. The
    central part of BlueSky consists of a server managing all simulations,
    normally together with a gui. The different modes for this are:

    - server-gui:       Start gui and simulation server
    - server-headless:  Start server without gui
    - client:           Start gui only, which can connect to an already
                        running server

    A BlueSky server can start one or more simulation processes, which run
    the actual simulations. These simulations can also be started completely
    separate from all other BlueSky functionality, in the detached mode.
    This is useful when calling BlueSky from within another Python
    script/program. The corresponding modes are:

    - sim:              The normal simulation process started by a BlueSky server
    - sim-detached:     An isolated simulation node, without networking
    """

    # Store keyword arguments passed to BlueSky during initialization
    kwargs = {}

    # Parse command-line arguments
    if "--detached" in sys.argv:
        mode = "sim-detached"
    elif "--sim" in sys.argv:
        mode = "sim"
    elif "--client" in sys.argv:
        mode = "client"
    elif "--headless" in sys.argv:
        mode = "server-headless"
    else:
        mode = "server-gui"

    # Determine if the BlueSky process should be discoverable (default is False)
    if "--discoverable" in sys.argv or "headless" in mode:
        kwargs["discovery"] = True

    # Check if alternate config file is passed
    if "--config-file" in sys.argv:
        idx = sys.argv.index("--config-file")
        try:
            kwargs["cfgfile"] = sys.argv[idx + 1]
        except IndexError:
            print("Missing argument for '--config-file', using default config file")

    # Check if default scenario file is passed
    if "--scenfile" in sys.argv:
        idx = sys.argv.index("--scenfile")
        try:
            kwargs["scnfile"] = sys.argv[idx + 1]
        except IndexError:
            print("Missing argument for '--scenfile', no scenario file will be loaded")

    # Catch Python module import errors
    try:
        # Initialize BlueSky modules
        bs.init(mode, **kwargs)

        # Only start a simulation node if called with --sim or --detached
        if mode == "sim":
            bs.sim.connect()
            bs.sim.run()
        elif mode == "sim-detached":
            bs.sim.run()

        # Start server if server-gui or server-headless is started here
        if mode == "server-gui":
            bs.server.start()
        elif mode == "server-headless":
            bs.server.run()

        # Start gui if client or main server/gui combination is started here
        if mode in ("client", "server-gui"):
            from bluesky.ui import qtgl
            qtgl.start(mode)

    # Give info on missing module
    except ImportError as error:
        # When ImportError gives different name than (pip) install needs,
        # also advise latest version
        missingmodules = {"OpenGL": "pyopengl and pyopengl-accelerate"}

        modulename = missingmodules.get(error.name) or error.name

        # Will crash program if ImportError source is unknown
        if modulename is None:
            raise error
        else:
            print("BlueSky needs Python package: {}".format(modulename))
            print("Install using e.g. 'pip install {}'".format(modulename))

    print('BlueSky normal end.')


if __name__ == "__main__":
    # Run main loop if BlueSky is called directly
    main()
