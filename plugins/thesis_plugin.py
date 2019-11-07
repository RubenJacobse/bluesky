"""
Restricted Airspace Area Plugin

Uses the AreaRestrictionManager class to keep track of restricted
areas and ensures that traffic will not enter those areas.

Current implementation is heavily work-in-progress and unstable.

© Ruben Jacobse, 2019
"""

# Local imports
import plugins.geovector as gv
from plugins.thesis.area_manager import AreaRestrictionManager

# Initialize the Manager object
AirspaceRestrictions = AreaRestrictionManager()

def _init_plugin():
    """Initialize the RAA plugin"""

    # Configuration parameters
    config = {
        "plugin_name": "RAA",
        "plugin_type": "sim",
        "update_interval": 1.0,
        "update": update,
        "preupdate": preupdate,
        "reset": reset,
        "remove": remove,
    }

    stackfunctions = {
        "RAA": [
            "RAA name, ON/OFF, [lat1,lon1,lat2,lon2,...]",
            "txt,onoff,[latlon,...]",
            create_area,
            "Create restricted airspace areas that are to be avoided by all traffic."],
        "DELRAA": [
            "DELRAA name",
            "txt",
            delete_area,
            "Delete a given restricted airspace area."],
        "RAACONF": [
            "RAACONF t_lookahead",
            "int",
            set_t_lookahead,
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

def preupdate():
    pass

def update():
    pass

def reset():
    AirspaceRestrictions.reset()

def remove():
    AirspaceRestrictions.remove()

def create_area(area_id, area_status, *coords):
    AirspaceRestrictions.create_area(area_id, area_status, coords)

def delete_area(area_id):
    AirspaceRestrictions.delete_area(area_id)

def set_t_lookahead(new_t_lookahead):
    AirspaceRestrictions.set_t_lookahead(new_t_lookahead)

def apply_area_avoidance():
    AirspaceRestrictions.apply_restrictions()
