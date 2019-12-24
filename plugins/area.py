""" BlueSky deletion area plugin. This plugin can use an area definition to
    delete aircraft that exit the area. Statistics on these flights can be
    logged with the FLSTLOG logger. """
import numpy as np
# Import the global bluesky objects. Uncomment the ones you need
from bluesky import traf, sim  #, settings, navdb, traf, sim, scr, tools
from bluesky.tools import datalog, areafilter, \
    TrafficArrays, RegisterElementParameters
from bluesky.tools import geo
from bluesky.tools.aero import ft
from bluesky import settings

# Log parameters for the flight statistics log
header = \
    ("Deletion time [s], "
     + "Callsign [-], "
     + "Spawn time [s], "
     + "Flight time [s], "
     + "Nominal Distance 2D [m],"
     + "Actual Distance 2D [m], "
    #  + "Actual Distance 3D [m], "
    #  + "Work Done [J], "
     + "Dist to last waypoint [m], "
     + "Latitude [deg], "
     + "Longitude [deg], "
    #  + "Altitude [m], "
    #  + "TAS [m/s], "
    #  + "Vertical Speed [m/s], "
    #  + "Heading [deg], "
     + "Time in conflict [s], "
     + "Time in LoS [s], "
     + "Time in resolution [s], "
     + "Num conflicts [-],"
     + "Num LoS [-]"
     # + "Origin Lat [deg], "
     # + "Origin Lon [deg], "
     # + "Destination Lat [deg], "
     # + "Destination Lon [deg], "
     # + "ASAS Active [bool], "
     # + "Pilot ALT [m], "
     # + "Pilot SPD (TAS) [m/s], "
     # + "Pilot HDG [deg], "
     # + "Pilot VS [m/s]"
     )

# Global data
area = None

### Initialization function of your plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.
def init_plugin():

    # Addtional initilisation code
    global area
    area = Area()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'AREA',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds.
        'update_interval': area.dt,

        # The update function is called after traffic is updated.
        'update':          area.update,
        }

    stackfunctions = {
        "AREA": [
            "AREA Shapename/OFF or AREA lat,lon,lat,lon,[top,bottom]",
            "[float/txt,float,float,float,alt,alt]",
            area.set_area,
            "Define experiment area (area of interest)"
        ],
        "TAXI": [
            "TAXI ON/OFF [alt] : OFF auto deletes traffic below 1500 ft",
            "onoff[,alt]",
            area.set_taxi,
            "Switch on/off ground/low altitude mode, prevents auto-delete at 1500 ft"
        ]
    }
    # init_plugin() should always return these two dicts.
    return config, stackfunctions

class Area(TrafficArrays):
    """ Traffic area: delete traffic when it leaves this area (so not when outside)"""
    def __init__(self):
        super(Area, self).__init__()
        # Parameters of area
        self.active = False
        self.dt     = 0.5     # [s] frequency of area check (simtime)
        self.name   = None
        self.swtaxi = True  # Default OFF: Doesn't do anything. See comments of set_taxi fucntion below.
        self.swtaxialt = 1500.  # Default OFF: Doesn't do anything. See comments of set_taxi fucntion below.

        # The FLST logger
        self.logger = datalog.crelog('FLSTLOG', None, header)

        with RegisterElementParameters(self):
            self.inside      = np.array([],dtype = np.bool) # In test area or not
            self.oldalt      = np.array([])
            self.distance2D  = np.array([])
            self.distance3D  = np.array([])
            self.work        = np.array([])
            self.create_time = np.array([])

    def create(self, n=1):
        super(Area, self).create(n)
        self.create_time[-n:] = sim.simt
        self.oldalt[-n:] = traf.alt[-n:]

    def update(self):
        ''' Update flight efficiency metrics
            2D and 3D distance [m], and work done (force*distance) [J] '''
        if self.swtaxi and not self.active:
            return

        resultantspd = np.sqrt(traf.gs * traf.gs + traf.vs * traf.vs)
        self.distance2D += self.dt * traf.gs
        self.distance3D += self.dt * resultantspd

        if settings.performance_model == 'openap':
            self.work += (traf.perf.thrust * self.dt * resultantspd)
        else:
            self.work += (traf.perf.Thr * self.dt * resultantspd)

        # Autodelete for descending with swTaxi:
        if not self.swtaxi:
            #delidxalt = np.where(traf.alt<self.swtaxialt)[0]
            delidxalt = np.where((self.oldalt>=self.swtaxialt)*(traf.alt<self.swtaxialt))[0]
            self.oldalt = traf.alt
        else:
            delidxalt = []



        # Find out which aircraft are currently inside the experiment area, and
        # determine which aircraft need to be deleted.
        inside = areafilter.checkInside(self.name, traf.lat, traf.lon, traf.alt)
        #delidx = np.intersect1d(np.where(np.array(self.inside)==True), np.where(np.array(inside)==False))
        delidx = np.where(np.array(self.inside)*(np.array(inside) == False))[0]
        self.inside = inside

        # Log flight statistics when for deleted aircraft
        if len(delidx) > 0:
            # Calculate nominal route distance and distance to last
            # waypoint at deletion time
            routedist = np.zeros(len(delidx))
            lastwpdist = np.zeros(len(delidx))
            for _, ac_idx in enumerate(delidx):
                routedist = sum(traf.ap.route[ac_idx].wpdistto) * 1852
                lastwpdist = geo.latlondist(traf.lat[ac_idx],
                                            traf.lon[ac_idx],
                                            traf.ap.route[ac_idx].wplat[-1],
                                            traf.ap.route[ac_idx].wplon[-1])

                if (self.create_time[ac_idx] >= 1800
                      and self.create_time[ac_idx] <= 9000):
                    self.logger.log(
                        f"{traf.id[ac_idx]}",
                        f"{self.create_time[ac_idx]}",
                        f"{(sim.simt - self.create_time[ac_idx]):.0f}",
                        f"{routedist:.0f}",
                        f"{self.distance2D[ac_idx]:.0f}",
                        # f"{self.distance3D[ac_idx]:.0f}",
                        # f"{self.work[ac_idx]:.0f}",
                        f"{lastwpdist:.0f}",
                        f"{traf.lat[ac_idx]:.4f}",
                        f"{traf.lon[ac_idx]:.4f}",
                        # f"{traf.alt[ac_idx]:.0f}",
                        # f"{traf.tas[ac_idx]:.0f}",
                        # f"{traf.vs[ac_idx]:.0f}",
                        # f"{traf.hdg[ac_idx]:.0f}",
                        f"{traf.asas.tot_time_inconf[ac_idx]:.0f}",
                        f"{traf.asas.tot_time_inlos[ac_idx]:.0f}",
                        f"{traf.asas.tot_time_inreso[ac_idx]:.0f}",
                        f"{traf.asas.num_tot_conf[ac_idx]}",
                        f"{traf.asas.num_tot_los[ac_idx]}",
                        # traf.ap.origlat[ac_idx],
                        # traf.ap.origlon[ac_idx],
                        # traf.ap.destlat[ac_idx],
                        # traf.ap.destlon[ac_idx],
                        # traf.asas.active[ac_idx],
                        # traf.pilot.alt[ac_idx],
                        # traf.pilot.tas[ac_idx],
                        # traf.pilot.vs[ac_idx],
                        # traf.pilot.hdg[ac_idx]
                    )
            # delete all aicraft in self.delidx
            traf.delete(delidx)


        # Delete all aicraft in self.delidxalt
        if len(delidxalt) > 0:
            traf.delete(list(delidxalt))

    def set_area(self, *args):
        ''' Set Experiment Area. Aicraft leaving the experiment area are deleted.
        Input can be exisiting shape name, or a box with optional altitude constrainsts.'''

        # if all args are empty, then print out the current area status
        if not args:
            return True, "Area is currently " + ("ON" if self.active else "OFF") + \
                         "\nCurrent Area name is: " + str(self.name)

        # start by checking if the first argument is a string -> then it is an area name
        if isinstance(args[0], str) and len(args)==1:
            if areafilter.hasArea(args[0]):
                # switch on Area, set it to the shape name
                self.name = args[0]
                self.active = True
                self.logger.start()
                return True, "Area is set to " + str(self.name)
            if args[0]=='OFF' or args[0]=='OF':
                # switch off the area
                areafilter.deleteArea(self.name)
                self.logger.reset()
                self.active = False
                self.name = None
                return True, "Area is switched OFF"

            # shape name is unknown
            return False, "Shapename unknown. " + \
                "Please create shapename first or shapename is misspelled!"
        # if first argument is a float -> then make a box with the arguments
        if isinstance(args[0],(float, int)) and 4<=len(args)<=6:
            self.active = True
            self.name = 'DELAREA'
            areafilter.defineArea(self.name, 'BOX', args[:4], *args[4:])
            self.logger.start()
            return True, "Area is ON. Area name is: " + str(self.name)

        return False,  "Incorrect arguments" + \
                       "\nAREA Shapename/OFF or\n Area lat,lon,lat,lon,[top,bottom]"

    def set_taxi(self, flag,alt=1500*ft):
        """ Taxi ON/OFF to autodelete below a certain altitude if taxi is off"""
        self.swtaxi = flag # True =  taxi allowed, False = autodelete below swtaxialt
        self.swtaxialt = alt
