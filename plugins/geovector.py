"""
Geovectoring: for an area define an allowed interval for each
component of the 3D speed vector (gs,trk,vs):

first argument is the are on whuch the geovector shoudl apply

Geovector is defined as:
 (  [ gsmin,gsmax ]  ) [kts]
 (  [trkmin,trkmax]  ) [deg]
 (  [ vsmin,vsmax ]  ) [fpm]
"""

# Third party imports
import numpy as np

# BlueSky imports
import bluesky as bs
from bluesky.tools import areafilter
from bluesky.tools.aero import vtas2cas,ft
from bluesky.tools.misc import degto180

# import plugins.thesis.area_manager
# from plugins.thesis.area_manager import SteeringMode

# A dictionary of areas with a geovector specification
geovecs = dict()


class GeoVector:
    ''' Geovector specification class. '''
    def __init__(self, areaname, gsmin, gsmax,
                 trkmin, trkmax,
                 vsmin, vsmax):
        self.areaname = areaname
        self.gsmin = gsmin
        self.gsmax = gsmax
        self.trkmin = trkmin
        self.trkmax = trkmax
        self.vsmin = vsmin
        self.vsmax = vsmax
        self.previnside = set()
        self.prevstatus = dict()

    def __str__(self):
        ''' Pretty printed geovector information. '''
        return f'gs: {self.gsmin}-{self.gsmax} [m/s], ' + \
            f'{self.trkmin}-{self.trkmax} trk[deg], ' + \
                f'{self.vsmin}-{self.vsmax} vs[m/s]'


def init_plugin():

    # Create an empty geovector list
    reset()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name'      : 'GEOVECTOR',
        'plugin_type'      : 'sim',
        'update_interval'  :  1.0,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        'update':          update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        'preupdate':       preupdate,

        # Reset all geovectors
        'reset':         reset
        }

    # Add two commands: GEOVECTOR to define a geovector for an area
    stackfunctions = {
        # Defining a geovector
        'GEOVECTOR': [
            'GEOVECTOR area,[gsmin,gsmax,trkmin,trkmax,vsmin,vsmax]',
            'txt,[spd,spd,hdg,hdg,vspd,vspd]',
            defgeovec,
            'Define a geovector for an area defined with the BOX,POLY(ALT) area commands']
        ,
        # Delete a geovector (same effect as using a geovector without  any values
        'DELGEOVECTOR': [
            'DELGEOVECTOR area',
            'txt',
            delgeovec,
            'Remove geovector from the area ']
        }
    # init_plugin() should always return these two dicts.
    return config, stackfunctions


### Periodic update functions that are called by the simulation. You can replace
### this by anything, so long as you communicate this in init_plugin

def preupdate(): # To be safe preupdate is used iso update
    applygeovec()
    return

def applygeovec():
    # Apply each geovector
    for areaname, vec in geovecs.items():
        if areafilter.hasArea(areaname):
            swinside  = areafilter.checkInside(areaname,
                                               bs.traf.lat,
                                               bs.traf.lon,
                                               bs.traf.alt)
            # Check if aircraft is in area resolution
            areareso = np.array([x == 2 for x
                                 in bs.traf._children[-1].control_mode_curr],
                                dtype=bool)
            # Array with boolean; True if inside geovector area and not in 
            # area resolution
            q = np.logical_and(swinside, np.logical_not(areareso))

            insids = set(np.array(bs.traf.id)[q])
            # print(f"t={bs.sim.simt}: {insids}")
            newids = insids - vec.previnside
            delids = vec.previnside - insids
            # Store LNAV/VNAV status of new aircraft
            for acid in newids:
                idx = bs.traf.id2idx(acid)
                vec.prevstatus[acid] = [bs.traf.swlnav[idx], bs.traf.swvnav[idx]]
                print(f"{acid}, LNAV {bs.traf.swlnav[idx]}; geovector on")

            # Revert aircraft who have exited the geovectored area to their original status
            for acid in delids:
                idx = bs.traf.id2idx(acid)
                if idx >= 0:
                    bs.traf.swlnav[idx], bs.traf.swvnav[idx] = vec.prevstatus.pop(acid)

                    # # TODO: Figure out why this does not work...
                    # # Fly direct to next waypoint on exiting geovector area
                    # bs.traf.swlnav[idx] = True  # Override LNAV on/off selector
                    # iwpid = bs.traf.ap.route[idx].findact(idx)
                    # wpname = bs.traf.ap.route[idx].wpname[iwpid]
                    # bs.traf.ap.route[idx].direct(idx,
                    #                              bs.traf.ap.route[idx].wpname[iwpid])
                    # print(f"{acid}, LNAV {bs.traf.swlnav[idx]} - next: {wpname}")

            vec.previnside = insids
            # -----Ground speed limiting
            # For now assume no wind:  so use tas as gs
            if vec.gsmin is not None:
                casmin = vtas2cas(np.ones(bs.traf.ntraf) * vec.gsmin, bs.traf.alt)
                usemin = vtas2cas(bs.traf.pilot.tas, bs.traf.alt) < casmin
                bs.traf.pilot.tas[swinside & usemin] = casmin[swinside & usemin]
                bs.traf.swvnav[swinside & usemin] = False

            if vec.gsmax is not None:
                casmax = vtas2cas(np.ones(bs.traf.ntraf) * vec.gsmax, bs.traf.alt)
                usemax = vtas2cas(bs.traf.pilot.tas, bs.traf.alt) > casmax
                bs.traf.pilot.tas[swinside & usemax] = casmax[swinside & usemax]
                bs.traf.swvnav[swinside & usemax] = False

            #------ Limit Track(so hdg)
            # Max track interval is 180 degrees to avoid ambiguity of what is inside the interval

            if None not in [vec.trkmin, vec.trkmax]:
                # Use degto180 to avodi problems for e.g interval [350,30]
                usemin = swinside & (degto180(bs.traf.pilot.hdg - vec.trkmin) < 0) # Left of minimum
                usemax = swinside & (degto180(bs.traf.pilot.hdg - vec.trkmax) > 0) # Right of maximum

                #print(usemin,usemax)
                bs.traf.swlnav[swinside & (usemin | usemax)] = False

                bs.traf.pilot.hdg[swinside & usemin] = vec.trkmin
                bs.traf.pilot.hdg[swinside & usemax] = vec.trkmax

            # -----Ground speed limiting
            # For now assume no wind:  so use tas as gs
            if vec.vsmin is not None:
                bs.traf.selvs[swinside & (bs.traf.vs < vec.vsmin)] = vec.vsmin
                bs.traf.swvnav[swinside & (bs.traf.vs < vec.vsmin)] = False
                # Activate V/S mode by using a slightly higher altitude than current values
                bs.traf.selalt[swinside & (bs.traf.vs < vec.vsmin)] = bs.traf.alt[swinside & (bs.traf.vs < vec.vsmin)] + \
                                                            np.sign(vec.vsmin)*200.*ft

            if vec.vsmax is not None:
                bs.traf.selvs[swinside & (bs.traf.vs > vec.vsmax)] = vec.vsmax
                bs.traf.swvnav[swinside & (bs.traf.vs < vec.vsmax)] = False
                # Activate V/S mode by using a slightly higher altitude than current values
                bs.traf.selalt[swinside & (bs.traf.vs > vec.vsmax)] = bs.traf.alt[swinside & (bs.traf.vs > vec.vsmax)] + \
                                                            np.sign(vec.vsmax)*200.*ft

    return

def update(): # Not used
    return

def reset():
    geovecs.clear() # [[area,gsmin,gsmax,trkin,trkmax,vsmin,vsmax]]
    return

### Other functions of your plug-in

def defgeovec(areaname="", spdmin=None, spdmax=None, trkmin=None, trkmax=None, vspdmin=None, vspdmax=None):
    #print ("defgeovec input=",areaname,gsmin,gsmax,trkmin,trkmax,vspdmin,vspdmax,sep="|")

    # We need an area to do anything
    if not areaname:
        return False, "We need an area"

    if not (spdmin or spdmax or (trkmin and trkmax) or vspdmin or vspdmax):
        if areaname in geovecs:
            return True, areaname + " uses " + str(geovecs[areaname])
        return False, "No geovector found for " + areaname

    # Remove old geovector for this area (if it exists)
    geovecs.pop(areaname, None)

    # Only add it if an interval is given
    if spdmin or spdmax or (trkmin and trkmax) or vspdmin or vspdmax:
        # Allow wrong order of min and max (note for hdg, as 355,10 is valid range!)
        if None not in [spdmin, spdmax]:
            gsmin = min(spdmin,spdmax)
            gsmax = max(spdmin,spdmax)
        else:
            gsmin = spdmin
            gsmax = spdmax

        if None not in [vspdmin, vspdmax]:
            vsmin = min(vspdmin,vspdmax)
            vsmax = max(vspdmin,vspdmax)
        else:
            vsmin = vspdmin
            vsmax = vspdmax

        # Add geovector to the dict of geovectors
        geovecs[areaname] = GeoVector(areaname,
                        gsmin, gsmax,
                        trkmin,trkmax,
                        vsmin, vsmax)

    return True

def delgeovec(area=""):
    # Delete geovector specification for area if it exists
    found = geovecs.pop(area)
    if not found:
        return False, "No geovector found for " + area

    return True, 'Deleted geovector specification for ' + area
