"""
Restricted Airspace Area Plugin

Uses the AreaRestrictionManager class to keep track of restricted
areas and ensures that traffic will not enter those areas.

Current implementation is heavily work-in-progress and unstable.

© Ruben Jacobse, 2019
"""

# Local imports
import plugins.geovector as gv
from plugins.thesis import AreaRestrictionManager


def init_plugin():
    """Initialize the RAA plugin"""

    # Addtional initilisation code
    areas = AreaRestrictionManager()

    # Configuration parameters
    config = {
        "plugin_name": "RAA",
        "plugin_type": "sim",
        "update_interval": 1.0,
        "update": areas.update,
        "preupdate": areas.preupdate,
        "reset": areas.reset,
        "remove": areas.remove,
    }

    stackfunctions = {
        "RAA": [
            "RAA name, ON/OFF, [lat1,lon1,lat2,lon2,...]",
            "txt,onoff,[latlon,...]",
            areas.create_area,
            "Create restricted airspace areas that are to be avoided by all traffic."],
        "DELRAA": [
            "DELRAA name",
            "txt",
            areas.delete_area,
            "Delete a given restricted airspace area."],
        "RAACONF": [
            "RAACONF t_lookahead",
            "int",
            areas.set_t_lookahead,
            "Set the lookahead time used for area avoidance in seconds."],
        'GEOVECTOR': [
            'GEOVECTOR area,[gsmin,gsmax,trkmin,trkmax,vsmin,vsmax]',
            'txt,[spd,spd,hdg,hdg,vspd,vspd]',
            gv.defgeovec,
            'Define a geovector for an area defined with the BOX,POLY(ALT) area commands'],
        'DELGEOVECTOR': [
            'DELGEOVECTOR area',
            'txt',
            gv.delgeovec,
            'Remove geovector from the area ']
    }

    return config, stackfunctions
