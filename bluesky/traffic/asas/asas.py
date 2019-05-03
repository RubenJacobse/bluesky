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

# Register default settings
settings.set_variable_defaults(asas_dt=1.0,
                               asas_dtlookahead=300.0,
                               asas_mar=1.2,
                               asas_pzr=5.0,
                               asas_pzh=1000.0,
                               asas_vmin=200.0,
                               asas_vmax=500.0)


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
                  "SWARM": Swarm}

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

        # All other ASAS variables are initialized in the reset function
        self.reset()

    def reset(self):
        """
        Reset all ASAS variables to their initial state. This function is
        used to both intialize and reset all ASAS variables.
        """

        super(ASAS, self).reset()

        # ASAS constructor
        self.cd_name      = "STATEBASED"
        self.cr_name      = "OFF"
        self.cd           = ASAS.cd_methods[self.cd_name]
        self.cr           = ASAS.cr_methods[self.cr_name]

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

    def toggle(self, flag=None):
        """
        Switch the ASAS module ON or OFF, and reset the conflict database.
        If no argument is provided, returns only the current state and does
        not perform database reset.
        """

        if flag is None:
            return True, "ASAS module is currently {}".format("ON" if self.swasas else "OFF")

        # Clear conflict list when switched off
        self.swasas = flag
        if not self.swasas:
            self.clear_conflict_database()
            self.inconf = False

        return True, "ASAS module set to: {}".format("ON" if self.swasas else "OFF")

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

        self.R  = radius * nm
        self.Rm = np.maximum(self.mar * self.R, self.Rm)

    def SetPZH(self, height=None):
        """
        Set the protected zone height in feet. If no argument is given,
        the current value is returned.
        """

        if height is None:
            return True, "ZONEDH [height (ft)]\n" \
                         + "Current PZ height: {:.2f} ft".format(self.dh / ft)

        self.dh  = height * ft
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

        self.Rm  = radius_margin * nm

    def SetPZHm(self, height_margin=None):
        """
        Set the protected zone height margin in feet. If no argument is
        given,the current value is returned.
        """

        if height_margin is None:
            return True, "RSZONEDH [height (ft)]\n" \
                         + "Current PZ height margin: {:.2f} ft".format(self.dhm / ft)

        if height_margin < self.dh / ft:
            return False, "PZ height margin may not be smaller than PZ height"

        self.dhm  = height_margin * ft

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
        Processes the RMETHH command. Sets swresovert = False
        """

        # Acceptable arguments for this command
        options = ["BOTH", "SPD", "HDG", "NONE", "ON", "OFF", "OF"]

        if value is None:
            return True, "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]" + \
                         "\nHorizontal resolution limitation is currently " + ("ON" if self.swresohoriz else "OFF") + \
                         "\nSpeed resolution limitation is currently " + ("ON" if self.swresospd else "OFF") + \
                         "\nHeading resolution limitation is currently " + ("ON" if self.swresohdg else "OFF")

        if str(value) not in options:
            return False, "RMETH Not Understood" + "\nRMETHH [ON / BOTH / OFF / NONE / SPD / HDG]"
        else:
            if value == "ON" or value == "BOTH":
                self.swresohoriz = True
                self.swresospd   = True
                self.swresohdg   = True
                self.swresovert  = False
            elif value == "OFF" or value == "OF" or value == "NONE":
                # Do NOT swtich off self.swresovert if value == OFF
                self.swresohoriz = False
                self.swresospd   = False
                self.swresohdg   = False
            elif value == "SPD":
                self.swresohoriz = True
                self.swresospd   = True
                self.swresohdg   = False
                self.swresovert  = False
            elif value == "HDG":
                self.swresohoriz = True
                self.swresospd   = False
                self.swresohdg   = True
                self.swresovert  = False

    def SetResoVert(self, value=None):
        """
        Processes the RMETHV command. Sets swresohoriz = False
        """

        # Acceptable arguments for this command
        options = ["NONE", "ON", "OFF", "OF", "V/S"]
        if value is None:
            return True, "RMETHV [ON / V/S / OFF / NONE]" + \
                         "\nVertical resolution limitation is currently " + ("ON" if self.swresovert else "OFF")

        if str(value) not in options:
            return False, "RMETV Not Understood" + "\nRMETHV [ON / V/S / OFF / NONE]"
        else:
            if value == "ON" or value == "V/S":
                self.swresovert  = True
                self.swresohoriz = False
                self.swresospd   = False
                self.swresohdg   = False
            elif value == "OFF" or value == "OF" or value == "NONE":
                # Do NOT swtich off self.swresohoriz if value == OFF
                self.swresovert  = False

    def SetResoFacH(self, value=None):
        """
        Set the horizontal resolution factor
        """

        if value is None:
            return True, ("RFACH [FACTOR]\nCurrent horizontal resolution factor is: %.1f" % self.resoFacH)

        self.resoFacH = np.abs(value)
        self.R  = self.R * self.resoFacH
        self.Rm = self.R * self.mar

        return True, "IMPORTANT NOTE: " + \
                     "\nCurrent horizontal resolution factor is: " + str(self.resoFacH) + \
                     "\nCurrent PZ radius:" + str(self.R / nm) + " NM" + \
                     "\nCurrent resolution PZ radius: " + str(self.Rm / nm) + " NM\n"

    def SetResoFacV(self, value=None):
        """
        Set the vertical resolution factor
        """

        if value is None:
            return True, ("RFACV [FACTOR]\nCurrent vertical resolution factor is: %.1f" % self.resoFacV)

        self.resoFacV = np.abs(value)
        self.dh  = self.dh * self.resoFacV
        self.dhm = self.dh * self.mar

        return True, "IMPORTANT NOTE: " + \
                     "\nCurrent vertical resolution factor is: " + str(self.resoFacV) + \
                     "\nCurrent PZ height:" + str(self.dh / ft) + " ft" + \
                     "\nCurrent resolution PZ height: " + str(self.dhm / ft) + " ft\n"

    def SetPrio(self, flag=None, priocode="FF1"):
        """
        Set the prio switch and the type of prio
        """

        if self.cr_name == "SSD":
            options = ["RS1","RS2","RS3","RS4","RS5","RS6","RS7","RS8","RS9"]
        else:
            options = ["FF1", "FF2", "FF3", "LAY1", "LAY2"]

        if flag is None:
            if self.cr_name == "SSD":
                return True, "PRIORULES [ON/OFF] [PRIOCODE]"  + \
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
                return True, "PRIORULES [ON/OFF] [PRIOCODE]"  + \
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

    def SetNoreso(self, noresoac=''):
        '''ADD or Remove aircraft that nobody will avoid.
        Multiple aircraft can be sent to this function at once '''
        if noresoac is '':
            return True, "NORESO [ACID]" + \
                         "\nCurrent list of aircraft nobody will avoid:" + \
                         str(self.noresolst)

        # Split the input into separate aircraft ids if multiple acids are given
        acids = noresoac.split(',') if len(noresoac.split(',')) > 1 else noresoac.split(' ')

        # Remove acids if they are already in self.noresolst. This is used to
        # delete aircraft from this list.
        # Else, add them to self.noresolst. Nobody will avoid these aircraft
        if set(acids) <= set(self.noresolst):
            self.noresolst = [x for x in self.noresolst if x not in set(acids)]
        else:
            self.noresolst.extend(acids)

        # active the switch, if there are acids in the list
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
                self.active[ac_idx] = True
            else:
                self.resoofflst.append(ac_id)
                self.active[ac_idx] = False

        # active the switch, if there are acids in the list
        self.swresooff = len(self.resoofflst) > 0

        return True, "RESOOFF is set for: {}".format(self.resoofflst)

    def SetVLimits(self, flag=None, spd=None):
        # Input is in knots
        if flag is None:
            return True, "ASAS limits in kts are currently [" + str(self.vmin * 3600 / 1852) + ";" + str(self.vmax * 3600 / 1852) + "]"

        if flag == "MAX":
            self.vmax = spd * nm / 3600.
        else:
            self.vmin = spd * nm / 3600.

    def create(self, n=1):
        """
        Create n new aircraft elements.
        """

        # Call actual create function
        super(ASAS, self).create(n)

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
        else:
            ac_id = bs.traf.id[ac_idx]
            if ac_id in self.resoofflst:
                self.resoofflst.remove(ac_id)

        super().delete(idx)

    def ResumeNav(self):
        """
        Decide for each aircraft in the conflict list whether the ASAS
        should be followed or not, based on if the aircraft pairs passed
        their CPA.
        """

        # Conflict pairs to be deleted
        delpairs = set()

        # Look at all conflicts, also the ones that are solved but CPA is yet to come
        for conflict in self.resopairs:
            idx1, idx2 = bs.traf.id2idx(conflict)
            
            # If the ownship aircraft is deleted remove its conflict from the list
            if idx1 < 0:
                delpairs.add(conflict)
                continue

            if idx2 >= 0:
                # Distance vector using flat earth approximation
                re = 6371000.
                dist = re * np.array([np.radians(bs.traf.lon[idx2] - bs.traf.lon[idx1]) *
                                      np.cos(0.5 * np.radians(bs.traf.lat[idx2] +
                                                              bs.traf.lat[idx1])),
                                      np.radians(bs.traf.lat[idx2] - bs.traf.lat[idx1])])

                # Relative velocity vector
                vrel = np.array([bs.traf.gseast[idx2] - bs.traf.gseast[idx1],
                                 bs.traf.gsnorth[idx2] - bs.traf.gsnorth[idx1]])

                # Check if conflict is past CPA
                past_cpa = np.dot(dist, vrel) > 0.0

                # hor_los:
                # Aircraft should continue to resolve until there is no horizontal
                # LOS. This is particularly relevant when vertical resolutions
                # are used.
                hdist = np.linalg.norm(dist)
                hor_los = hdist < self.R

                # Bouncing conflicts:
                # If two aircraft are getting in and out of conflict continously,
                # then they it is a bouncing conflict. ASAS should stay active until
                # the bouncing stops.
                is_bouncing = abs(bs.traf.trk[idx1] - bs.traf.trk[idx2]) < 30.0 and hdist < self.Rm

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 >= 0 and (not past_cpa or hor_los or is_bouncing):
                if bs.traf.id[idx1] not in self.resoofflst:
                    # Enable ASAS for this aircraft
                    self.active[idx1] = True
                else:
                    self.active[idx1] = False
            else:
                # Switch ASAS off for ownship
                self.active[idx1] = False

                # Waypoint recovery after conflict: Find the next active waypoint
                # and send the aircraft to that waypoint.
                iwpid = bs.traf.ap.route[idx1].findact(idx1)
                if iwpid != -1:  # To avoid problems if there are no waypoints
                    bs.traf.ap.route[idx1].direct(idx1, bs.traf.ap.route[idx1].wpname[iwpid])

                # If conflict is solved, remove it from the resopairs list
                delpairs.add(conflict)

        # Remove pairs from the list that are past CPA or have deleted aircraft
        self.resopairs -= delpairs

    @timed_function('asas', dt=settings.asas_dt)
    def update(self, dt):
        if not self.swasas or bs.traf.ntraf == 0:
            return

        # Conflict detection
        self.confpairs, self.lospairs, self.inconf, self.tcpamax, \
            self.qdr, self.dist, self.dcpa, self.tcpa, self.tLOS = \
            self.cd.detect(bs.traf, bs.traf, self.R, self.dh, self.dtlookahead)

        # Conflict resolution if there are conflicts
        if self.confpairs:
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

        self.ResumeNav()

        # iconf0 = np.array(self.iconf)
        #
