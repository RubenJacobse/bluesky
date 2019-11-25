"""
This module implements the Airborne Separation Assurance System. It uses
conflict detection and conflict resolution methods defined in separate
modules.
"""

# Python imports
try:
    from collections.abc import Collection
except ImportError:
    # In python <3.3 collections.abc doesn't exist
    from collections import Collection

# Third-party imports
import numpy as np

# BlueSky imports
import bluesky as bs
from bluesky import settings
from bluesky.tools import geo, datalog
from bluesky.tools.simtime import timed_function
from bluesky.tools.aero import ft, nm
from bluesky.tools.trafficarrays import TrafficArrays, RegisterElementParameters

# Import default conflict detection method
from . import StateBasedCD

# Import default conflict resolution methods
from . import DoNothing
from . import Eby
from . import MVP
from . import Swarm
from . import SSD
from . import LF
from . import LFFT
from . import Swarm_alt
from . import Swarm_v3
from . import MVP_PRIO

# Register default settings
settings.set_variable_defaults(asas_dt=1.0,
                               asas_dtlookahead=300.0,
                               asas_mar=1.05,
                               asas_pzr=5.0,
                               asas_pzh=1000.0,
                               asas_vmin=200.0,
                               asas_vmax=500.0)

# Header for logfile written by ASAS module
ASASLOG_HEADER = ("simt [s], "
                  + "ac1, "
                  + "ac2, "
                  + "LoS [-], "
                  + "Sev [-], "
                  + "duration [-]")
ASASPOS_HEADER = ("simt [s], "
                  + "LoS [-], "
                  + "lat [deg], "
                  + "lon [deg]")


class ASAS(TrafficArrays):
    """
    Class that implements the Airborne Separation Assurance System for
    conflict detection and resolution. Maintains a conflict database,
    and links to external CD and CR methods.
    """

    # Dictionary of conflict detection methods
    cd_methods = {"STATEBASED": StateBasedCD}

    # Dictionary of conflict resolution methods
    cr_methods = {"OFF": DoNothing,
                  "MVP": MVP,
                  "EBY": Eby,
                  "SWARM": Swarm,
                  "SWARM-V2": Swarm_alt,
                  "SWARM-V3": Swarm_v3,
                  "LF": LF,
                  "LFFT": LFFT,
                  "MVP-PRIO": MVP_PRIO}  # MVP + priority - experimental

    # The SSD method requires the pyclipper module for its visualizations
    if SSD.loaded_pyclipper():
        cr_methods["SSD"] = SSD

    @classmethod
    def add_conflict_detection_method(cls, name, module):
        """
        Add conflict detection method 'name' defined in 'module' to the ASAS
        class variable that tracks all available conflict detection methods.
        """

        cls.cd_methods[name] = module

    @classmethod
    def add_conflict_resolution_method(cls, name, module):
        """
        Add conflict resolution method 'name' defined in 'module' to the ASAS
        class variable that tracks all available conflict resolution methods.
        """

        cls.cr_methods[name] = module

    def __init__(self):
        super(ASAS, self).__init__()

        with RegisterElementParameters(self):
            # ASAS info per aircraft:
            self.inconf = np.array([], dtype=bool)  # In-conflict flag
            self.tcpamax = np.array([])  # Maximum time to CPA for aircraft in conflict
            self.active = np.array([], dtype=bool)  # whether the autopilot follows ASAS or not
            self.trk = np.array([])  # heading provided by the ASAS [deg]
            self.tas = np.array([])  # speed provided by the ASAS (eas) [m/s]
            self.alt = np.array([])  # alt provided by the ASAS [m]
            self.vs = np.array([])  # vspeed provided by the ASAS [m/s]

        # Create a new conflict logger
        self.conf_logger = datalog.crelog("ASASLOG", None, ASASLOG_HEADER)
        self.conf_logger.start()
        self.pos_logger = datalog.crelog("ASASPOS", None, ASASPOS_HEADER)
        self.pos_logger.start()

        # All other ASAS variables are initialized in the reset function
        self.reset()

    def reset(self):
        """
        Reset all ASAS variables to their initial state. This function is
        used to both intialize and reset all ASAS variables.
        """

        super(ASAS, self).reset()

        # ASAS constructor
        self.cd_name = "STATEBASED"
        self.cr_name = "OFF"
        self.cd = ASAS.cd_methods[self.cd_name]
        self.cr = ASAS.cr_methods[self.cr_name]

        self.dtasas       = settings.asas_dt
        self.dtlookahead  = settings.asas_dtlookahead # [s] lookahead time
        self.mar          = settings.asas_mar         # [-] Safety margin for evasion
        self.R            = settings.asas_pzr * nm    # [m] Horizontal separation minimum for detection
        self.dh           = settings.asas_pzh * ft    # [m] Vertical separation minimum for detection
        self.Rm           = self.R * self.mar         # [m] Horizontal separation minimum for resolution
        self.dhm          = self.dh * self.mar        # [m] Vertical separation minimum for resolution
        self.swasas       = True                      # [-] whether to perform CD&R

        self.vmin         = settings.asas_vmin * nm / 3600. # [m/s] Minimum ASAS velocity
        self.vmax         = settings.asas_vmax * nm / 3600. # [m/s] Maximum ASAS velocity
        self.vsmin        = -3000. / 60. * ft               # [m/s] Minimum ASAS vertical speed
        self.vsmax        = 3000. / 60. * ft                # [m/s] Maximum ASAS vertical speed

        self.swresohoriz  = True    # [-] switch to limit resolution to the horizontal direction
        self.swresospd    = False   # [-] switch to use only speed resolutions (works with swresohoriz = True)
        self.swresohdg    = False   # [-] switch to use only heading resolutions (works with swresohoriz = True)
        self.swresovert   = False   # [-] switch to limit resolution to the vertical direction
        self.swresocoop   = False   # [-] switch to limit resolution magnitude to half (cooperative resolutions)

        self.swprio       = False   # [-] switch to activate priority rules for conflict resolution
        self.priocode     = "FF1"   # [-] Code of the priority rule that is to be used (FF1, FF2, FF3, LAY1, LAY2)

        self.swnoreso     = False   # [-] switch to activate the NORESO command. Nobody will avoid conflicts with  NORESO aircraft
        self.noresolst    = []      # [-] list for NORESO command. Nobody will avoid conflicts with aircraft in this list

        self.swresooff    = False   # [-] switch to active the RESOOFF command. RESOOFF aircraft will NOT avoid other aircraft. Opposite of NORESO command.
        self.resoofflst   = []      # [-] list for the RESOOFF command. These aircraft will not do conflict resolutions.

        self.resoFacH     = 1.0     # [-] set horizontal resolution factor (1.0 = 100%)
        self.resoFacV     = 1.0     # [-] set horizontal resolution factor (1.0 = 100%)

        # ASAS-visualization on SSD
        self.asasn        = np.array([])    # [m/s] North resolution speed from ASAS
        self.asase        = np.array([])    # [m/s] East resolution speed from ASAS
        self.asaseval     = False           # [-] Whether target resolution is calculated or not

        self.clear_conflict_database()

        # NOTE: Should this also be moved to inside clear_conflict_database()?
        self.dcpa = np.array([])  # CPA distance

    def clear_conflict_database(self):
        """
        Clear the ASAS conflict database and reset the intial state.
        """

        self.confpairs = list()  # Conflict pairs detected in the current timestep (used for resolving)
        self.confpairs_unique = set()  # Unique conflict pairs (a, b) = (b, a) are merged
        self.resopairs = set()  # Resolved (when RESO is on) conflicts that are still before CPA
        self.lospairs = list()  # Current loss of separation pairs
        self.lospairs_unique = set()  # Unique LOS pairs (a, b) = (b, a) are merged
        self.confpairs_all = list()  # All conflicts since simt=0
        self.lospairs_all = list()  # All losses of separation since simt=0

        # Conflict time and geometry data per conflict pair
        self.tcpa = np.array([])  # Time to CPA
        self.tLOS = np.array([])  # Time to start LoS
        self.qdr = np.array([])  # Bearing from ownship to intruder
        self.dist = np.array([])  # Horizontal distance between ""

        # Dictionaries used for logging
        self.conf_tracker = {}
        self.los_tracker = {}

    def create(self, n=1):
        """
        Create n new aircraft elements.
        """

        # Call actual create function
        super(ASAS, self).create(n)

        # On creation, no asas calculation have been performed, so set
        # track, tas and altitude parameters equal to those of bs.traf
        self.trk[-n:] = bs.traf.trk[-n:]
        self.tas[-n:] = bs.traf.tas[-n:]
        self.alt[-n:] = bs.traf.alt[-n:]

    def delete(self, idx):
        """
        Delete one or more aircraft elements
        """

        if isinstance(idx, Collection):
            idx = np.sort(idx)

            for ac_idx in idx:
                ac_id = bs.traf.id[ac_idx]
                if ac_id in self.resoofflst:
                    self.resoofflst.remove(ac_id)
                if ac_id in self.noresolst:
                    self.noresolst.remove(ac_id)
        else:
            ac_id = bs.traf.id[idx]
            if ac_id in self.resoofflst:
                self.resoofflst.remove(ac_id)
            if ac_id in self.noresolst:
                self.noresolst.remove(ac_id)

        super(ASAS, self).delete(idx)

    def toggle(self, flag=None):
        """
        Switch the ASAS module ON or OFF, and reset the conflict database.
        If no argument is provided, returns only the current state and does
        not perform database reset.
        """

        if flag is None:
            return True, "ASAS module is currently {}".format("ON" if self.swasas else "OFF")

        # When ASAS is switched off, reset the conflict database and set all
        # elements of self.inconf to False.
        self.swasas = flag
        if not self.swasas:
            self.clear_conflict_database()
            self.inconf = self.inconf & False

        return True, "ASAS module set to: {}".format("ON" if self.swasas else "OFF")

    def SetCDmethod(self, method=""):
        """
        Set the ASAS conflict detection method. If no argument is given,
        the current method and a list of available methods is printed.
        """

        if not method:
            return True, "Current CD method: {}\nAvailable CD methods: {}"\
                .format(self.cd_name, str.join(", ", list(ASAS.cd_methods.keys())))

        if method not in ASAS.cd_methods:
            return False, "{} doesn't exist.\nAvailable CD methods: {}"\
                .format(method, str.join(", ", list(ASAS.cd_methods.keys())))

        self.cd_name = method
        self.cd = ASAS.cd_methods[method]

        # Reset the database for use by the new method
        self.clear_conflict_database()

    def SetCRmethod(self, method=""):
        """
        Set the ASAS conflict resolution method. If no argument is given,
        the current method and a list of available methods is printed.
        """

        if not method:
            return True, "Current CR method: {}\nAvailable CR methods: {}"\
                .format(self.cr_name, str.join(", ", list(ASAS.cr_methods.keys())))

        if method not in ASAS.cr_methods:
            return False, "{} doesn't exist.\nAvailable CR methods: {}"\
                .format(method, str.join(", ", list(ASAS.cr_methods.keys())))

        self.cr_name = method
        self.cr = ASAS.cr_methods[method]
        self.cr.start(self)

    def SetPZR(self, radius=None):
        """
        Set the protected zone radius in Nautical Miles. If no argument
        is given, the current value is returned.
        """

        if radius is None:
            return True, "ZONER [radius (nm)]\n" \
                         + "Current PZ radius: {:.2f} NM".format(self.R / nm)

        self.R = radius * nm
        self.Rm = np.maximum(self.mar * self.R, self.Rm)

    def SetPZH(self, height=None):
        """
        Set the protected zone height in feet. If no argument is given,
        the current value is returned.
        """

        if height is None:
            return True, "ZONEDH [height (ft)]\n" \
                         + "Current PZ height: {:.2f} ft".format(self.dh / ft)

        self.dh = height * ft
        self.dhm = np.maximum(self.mar * self.dh, self.dhm)

    def SetPZRm(self, radius_margin=None):
        """
        Set the protected zone radius margin in Nautical Miles. If no
        argument is given, the current value is returned.
        """

        if radius_margin is None:
            return True, "RSZONER [radius (nm)]\n" \
                         + "Current PZ radius margin: {:.2f} NM".format(self.Rm / nm)

        if radius_margin < self.R / nm:
            return False, "PZ radius margin may not be smaller than PZ radius"

        self.Rm = radius_margin * nm

    def SetPZHm(self, height_margin=None):
        """
        Set the protected zone height margin in feet. If no argument is
        given, the current value is returned.
        """

        if height_margin is None:
            return True, "RSZONEDH [height (ft)]\n" \
                         + "Current PZ height margin: {:.2f} ft".format(self.dhm / ft)

        if height_margin < self.dh / ft:
            return False, "PZ height margin may not be smaller than PZ height"

        self.dhm = height_margin * ft

    def SetDtLook(self, detection_time=None):
        """
        Set the conflict detection look-ahead time in seconds. If no argument
        is given, the current value is returned.
        """

        if detection_time is None:
            return True, "DTLOOK [time]\nCurrent value: {:.1f} sec".format(self.dtlookahead)

        self.dtlookahead = detection_time
        self.clear_conflict_database()

    def SetDtNoLook(self, detection_interval=None):
        """
        Set the interval for conflict detection in seconds. If no argument
        is given, the current value is returned.
        """

        if detection_interval is None:
            return True, "DTNOLOOK [time]\nCurrent value: {:.1f} sec".format(self.dtasas)

        self.dtasas = detection_interval

    def SetResoHoriz(self, value=None):
        """
        Processes the RMETHH command. Sets vertical resolution flag to False
        if horizontal resolution method is not NONE or OFF. If no argument
        is given, the current settings are returned.
        """

        # Acceptable arguments for this command
        options = ["BOTH", "SPD", "HDG", "NONE", "ON", "OFF", "OF"]

        if value is None:
            return True, "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]" \
                         + "\nHorizontal resolution limitation is currently " \
                         + ("ON" if self.swresohoriz else "OFF") \
                         + "\nSpeed resolution limitation is currently " \
                         + ("ON" if self.swresospd else "OFF") \
                         + "\nHeading resolution limitation is currently " \
                         + ("ON" if self.swresohdg else "OFF")

        if str(value) not in options:
            return False, "RMETHH command not understood, use:" \
                          + "\nRMETHH [ON / BOTH / OFF / NONE / SPD / HDG]"
        else:
            if value in ("ON", "BOTH"):
                self.swresohoriz = True
                self.swresospd = True
                self.swresohdg = True
                self.swresovert = False
            elif value in ("OFF", "OF", "NONE"):
                # Do NOT switch off self.swresovert if value == OFF
                self.swresohoriz = False
                self.swresospd = False
                self.swresohdg = False
            elif value == "SPD":
                self.swresohoriz = True
                self.swresospd = True
                self.swresohdg = False
                self.swresovert = False
            elif value == "HDG":
                self.swresohoriz = True
                self.swresospd = False
                self.swresohdg = True
                self.swresovert = False

    def SetResoVert(self, value=None):
        """
        Processes the RMETHV command. Sets horizontal resolution flag to False
        if the vertical resolution method is not NONE or OFF. If no argument
        is given, the current settings are returned.
        """

        # Acceptable arguments for this command
        options = ["NONE", "ON", "OFF", "OF", "V/S"]

        if value is None:
            return True, "RMETHV [ON / V/S / OFF / NONE]" \
                         + "\nVertical resolution limitation is currently " \
                         + ("ON" if self.swresovert else "OFF")

        if str(value) not in options:
            return False, "RMETHV command not understood, use:" \
                          + "\nRMETHV [ON / V/S / OFF / NONE]"
        else:
            if value in ("ON", "V/S"):
                self.swresovert = True
                self.swresohoriz = False
                self.swresospd = False
                self.swresohdg = False
            elif value in ("OFF", "OF", "NONE"):
                # Do NOT switch off self.swresohoriz if value == OFF
                self.swresovert = False

    def SetResoFacH(self, value=None):
        """
        Set the horizontal resolution factor. If no argument is given, the
        current setting is returned.
        """

        if value is None:
            return True, ("RFACH [FACTOR]\nCurrent horizontal resolution factor is: {:.1f}"
                          .format(self.resoFacH))

        self.resoFacH = np.abs(value)
        self.R = self.R * self.resoFacH
        self.Rm = self.R * self.mar

        return True, "IMPORTANT NOTE: " \
                     + "\nCurrent horizontal resolution factor is: {}".format(self.resoFacH) \
                     + "\nCurrent PZ radius: {} NM".format(self.R / nm) \
                     + "\nCurrent resolution PZ radius: {} NM\n".format(self.Rm / nm)

    def SetResoFacV(self, value=None):
        """
        Set the vertical resolution factor. If no argument is given, the
        current setting is returned.
        """

        if value is None:
            return True, ("RFACV [FACTOR]\nCurrent vertical resolution factor is: {:.1f}"
                          .format(self.resoFacV))

        self.resoFacV = np.abs(value)
        self.dh = self.dh * self.resoFacV
        self.dhm = self.dh * self.mar

        return True, "IMPORTANT NOTE: " \
                     + "\nCurrent vertical resolution factor is: {}".format(self.resoFacV) \
                     + "\nCurrent PZ height: {} ft".format(self.dh / ft) \
                     + "\nCurrent resolution PZ height: {} ft\n".format(self.dhm / ft)

    def SetPrio(self, flag=None, priocode="FF1"):
        """
        Set the prio switch and the type of prio
        """

        if self.cr_name == "SSD":
            options = ["RS1", "RS2", "RS3", "RS4", "RS5", "RS6", "RS7", "RS8", "RS9"]
        else:
            options = ["FF1", "FF2", "FF3", "LAY1", "LAY2"]

        if flag is None:
            if self.cr_name == "SSD":
                return True, "PRIORULES [ON/OFF] [PRIOCODE]" + \
                             "\nAvailable priority codes: " + \
                             "\n     RS1:  Shortest way out" + \
                             "\n     RS2:  Clockwise turning" + \
                             "\n     RS3:  Heading first, RS1 second" + \
                             "\n     RS4:  Speed first, RS1 second" + \
                             "\n     RS5:  Shortest from target" + \
                             "\n     RS6:  Rules of the air" + \
                             "\n     RS7:  Sequential RS1" + \
                             "\n     RS8:  Sequential RS5" + \
                             "\n     RS9:  Counterclockwise turning" + \
                             "\nPriority is currently " + ("ON" if self.swprio else "OFF") + \
                             "\nPriority code is currently: " + str(self.priocode)
            else:
                return True, "PRIORULES [ON/OFF] [PRIOCODE]" + \
                             "\nAvailable priority codes: " + \
                             "\n     FF1:  Free Flight Primary (No Prio) " + \
                             "\n     FF2:  Free Flight Secondary (Cruising has priority)" + \
                             "\n     FF3:  Free Flight Tertiary (Climbing/descending has priority)" + \
                             "\n     LAY1: Layers Primary (Cruising has priority + horizontal resolutions)" + \
                             "\n     LAY2: Layers Secondary (Climbing/descending has priority + horizontal resolutions)" + \
                             "\nPriority is currently " + ("ON" if self.swprio else "OFF") + \
                             "\nPriority code is currently: " + str(self.priocode)
        self.swprio = flag

        if priocode not in options:
            return False, "Priority code Not Understood. Available Options: " + str(options)
        else:
            self.priocode = priocode

    def SetNoreso(self, ac_no_reso=""):
        """
        ADD or Remove aircraft that nobody will avoid.
        Multiple aircraft can be sent to this function at once
        """

        if ac_no_reso is "":
            return True, "NORESO [ACID]" + \
                         "\nCurrent list of aircraft nobody will avoid:" + \
                         str(self.noresolst)

        # Split the input into separate aircraft ids if multiple acids are given
        ac_ids = (ac_no_reso.split(",") if len(ac_no_reso.split(",")) > 1
                  else ac_no_reso.split(" "))

        # Remove acids if they are already in self.noresolst. This is used to
        # delete aircraft from this list.
        # Else, add them to self.noresolst. Nobody will avoid these aircraft
        if set(ac_ids) <= set(self.noresolst):
            self.noresolst = [x for x in self.noresolst if x not in set(ac_ids)]
        else:
            self.noresolst.extend(ac_ids)

        # Activate the switch, if there are acids in the list
        self.swnoreso = len(self.noresolst) > 0

    def SetResooff(self, ac_reso_off=""):
        """
        ADD or Remove aircraft that will not avoid anybody else
        """

        if ac_reso_off is "":
            return True, "RESOOFF [ACID]" + \
                         "\nCurrent list of aircraft that will not avoid anybody:" + \
                         str(self.resoofflst)

        # Split the input into separate aircraft ids if multiple ids are given
        ac_ids = ac_reso_off.split(',') if len(ac_reso_off.split(',')) > 1 else ac_reso_off.split(' ')

        # Remove ids if they are already in self.resoofflst. This is used to
        # delete aircraft from this list.
        # Else, add them to self.resoofflst. These aircraft will not avoid anybody
        for ac_id in ac_ids:
            ac_idx = bs.traf.id2idx(ac_id)
            if ac_id in self.resoofflst:
                self.resoofflst.remove(ac_id)
            else:
                self.resoofflst.append(ac_id)
                self.active[ac_idx] = False

        # active the switch, if there are acids in the list
        self.swresooff = len(self.resoofflst) > 0

    def SetVLimits(self, flag=None, spd=None):
        """ """

        # Input is in knots
        if flag is None:
            return True, "ASAS limits are currently [{};{}] kts" \
                         .format(str(self.vmin * 3600 / 1852),
                                 str(self.vmax * 3600 / 1852))

        if flag == "MAX":
            self.vmax = spd * nm / 3600.
        else:
            self.vmin = spd * nm / 3600.

    @timed_function('asas', dt=settings.asas_dt)
    def update(self, dt):
        if not self.swasas or bs.traf.ntraf == 0:
            return

        # BlueSky does not signal start of new scenario, thus always check
        if self.conf_logger.scenname != bs.stack.get_scenname():
            self.conf_logger.reset()
            self.conf_logger.start()
        if self.pos_logger.scenname != bs.stack.get_scenname():
            self.pos_logger.reset()
            self.pos_logger.start()

        # Conflict detection
        self.confpairs, self.lospairs, self.inconf, self.tcpamax, \
            self.qdr, self.dist, self.dcpa, self.tcpa, self.tLOS = \
            self.cd.detect(bs.traf, bs.traf, self.R, self.dh, self.dtlookahead)

        # Conflict resolution only if there are conflicts or if swarming /
        # leader-following with follow through is used (does not require a
        # conflict for resolution)
        if self.confpairs or self.cr_name in ["SWARM-V2", "LFFT"]:
            self.cr.resolve(self, bs.traf)

        # Add new conflicts to resopairs and confpairs_all and new losses to lospairs_all
        self.resopairs.update(self.confpairs)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        confpairs_unique = {frozenset(pair) for pair in self.confpairs}
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}

        self.confpairs_all.extend(confpairs_unique - self.confpairs_unique)
        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)

        # Update confpairs_unique and lospairs_unique
        self.confpairs_unique = confpairs_unique
        self.lospairs_unique = lospairs_unique

        # Update conflict and los tracker variables
        if bs.sim.simt >= 1800 and bs.sim.simt <= 16200:
            for pair in confpairs_unique:
                if pair not in self.conf_tracker.keys():
                    self.conf_tracker[pair] = {"duration": 1}
                else:
                    self.conf_tracker[pair]["duration"] += 1

            for pair in lospairs_unique:
                pair_idx = self.confpairs.index(tuple(pair))
                severity = (self.R - self.dist[pair_idx]) / self.R
                if pair not in self.los_tracker.keys():
                    self.los_tracker[pair] = {"duration": 1,
                                              "severity": severity}
                else:
                    self.los_tracker[pair]["duration"] += 1
                    if severity < self.los_tracker[pair]["severity"]:
                        self.los_tracker[pair]["severity"] = severity

        self.resume_navigation()

        # Pairs that are no longer in conflict or los are logged
        # and deleted from the tracker dictionaries
        if bs.sim.simt >= 1800 and bs.sim.simt <= 16200:
            for pair in list(self.conf_tracker.keys()):
                if pair not in confpairs_unique:
                    self.conf_logger.log(tuple(pair)[0],
                                         tuple(pair)[1],
                                         0,
                                         0,
                                         self.conf_tracker[pair]["duration"])
                    del self.conf_tracker[pair]

            for pair in list(self.los_tracker.keys()):
                if pair not in lospairs_unique:
                    self.conf_logger.log(tuple(pair)[0],
                                         tuple(pair)[1],
                                         1,
                                         self.los_tracker[pair]["severity"],
                                         self.los_tracker[pair]["duration"])
                    del self.los_tracker[pair]

        # Only log conflict positions for t between 1800 and 3600 seconds
        if bs.sim.simt >= 1800 and bs.sim.simt <= 3600:  # 1800:
            for (ac1, ac2) in self.confpairs_unique:
                idx1 = bs.traf.id2idx(ac1)
                idx2 = bs.traf.id2idx(ac2)

                is_los = int((ac1, ac2) in self.lospairs)  # Either 0 or 1

                # Log LoS status and locations of both aircraft
                self.pos_logger.log(is_los,
                                         f"{bs.traf.lat[idx1]:.4f} ",
                                         f"{bs.traf.lon[idx1]:.4f}")
                self.pos_logger.log(is_los,
                                         f"{bs.traf.lat[idx2]:.4f} ",
                                         f"{bs.traf.lon[idx2]:.4f}")

    def resume_navigation(self):
        """
        Decide for each aircraft in the conflict list whether the ASAS
        should be followed or not, based on if the aircraft pairs passed
        their CPA.
        """

        # Conflict pairs to be deleted
        delpairs = set()
        changeactive = dict()

        # Look at all conflicts, also the ones that are solved but CPA is yet to come
        for conflict_pair in self.resopairs:
            (id1, id2) = conflict_pair
            idx1 = bs.traf.id2idx(id1)
            idx2 = bs.traf.id2idx(id2)

            # If the ownship aircraft is deleted remove its conflict from the list
            if idx1 < 0:
                delpairs.add(conflict_pair)
                continue

            if idx2 >= 0:
                # Distance vector using spherical earth qdr and distance
                qdr, dist_nm = geo.qdrdist(bs.traf.lat[idx1], bs.traf.lon[idx1],
                                           bs.traf.lat[idx2], bs.traf.lon[idx2])

                dist = dist_nm * 1852.0 * np.array([np.sin(np.radians(qdr)),
                                                    np.cos(np.radians(qdr))])

                # Relative velocity vector
                vrel = np.array([bs.traf.gseast[idx2] - bs.traf.gseast[idx1],
                                 bs.traf.gsnorth[idx2] - bs.traf.gsnorth[idx1]])

                # Check if conflict is past CPA
                is_past_cpa = np.dot(dist, vrel) > 0.0

                # Aircraft should continue to resolve until there is no horizontal
                # LOS. This is particularly relevant when vertical resolutions
                # are used.
                hor_dist = np.linalg.norm(dist)
                is_hor_los = hor_dist < self.R

                # Bouncing conflicts:
                # If two aircraft are getting in and out of conflict continously,
                # then they it is a bouncing conflict. ASAS should stay active until
                # the bouncing stops.
                is_bouncing = (abs(bs.traf.trk[idx1] - bs.traf.trk[idx2]) < 30.0
                               and hor_dist < self.Rm)

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not is_past_cpa or is_hor_los or is_bouncing):
                if bs.traf.id[idx1] not in self.resoofflst:
                    # Enable ASAS for this aircraft
                    changeactive[idx1] = True
                else:
                    changeactive[idx1] = False
            else:
                # Switch ASAS off for ownship if there are no other conflicts
                # that this aircraft is involved in.
                changeactive[idx1] = changeactive.get(idx1, False)

                # If conflict is solved, remove it from the resopairs list
                delpairs.add(conflict_pair)

        for idx, active in changeactive.items():
            # Loop a second time: this is to avoid that ASAS resolution is
            # turned off for an aircraft that is involved simultaneously in
            # multiple conflicts, where the first, but not all conflicts are
            # resolved.
            self.active[idx] = active
            if not active and not bs.traf.id[idx] in self.resoofflst:
                # Waypoint recovery after conflict: Find the next active waypoint
                # and send the aircraft to that waypoint.
                iwpid = bs.traf.ap.route[idx].findact(idx)
                if iwpid != -1:  # To avoid problems if there are no waypoints
                    bs.traf.ap.route[idx].direct(idx, bs.traf.ap.route[idx].wpname[iwpid])

        # Remove pairs that are past CPA or have deleted aircraft from the list
        self.resopairs -= delpairs
