""" Classes that derive from TrafficArrays (like Traffic) get automated create,
    delete, and reset functionality for all registered child arrays."""
# -*- coding: utf-8 -*-
try:
    from collections.abc import Collection
except ImportError:
    # In python <3.3 collections.abc doesn't exist
    from collections import Collection
import numpy as np

defaults = {"float": 0.0, "int": 0, "bool": False, "S": "", "str": ""}


class RegisterElementParameters():
    """ Class to use in 'with'-syntax. This class automatically
        calls for the MakeParameterLists function of the parent
        TrafficArrays object, with all parameters defined in 'with'. """

    def __init__(self, parent):
        # On initialization store the parent object as an attribute
        self._parent = parent

    def __enter__(self):
        # On entering the context only the parent's attributes are present,
        # store the keys of these attributes as a set().
        self.keys0 = set(self._parent.__dict__.keys())

    def __exit__(self, type, value, tb):
        # On exiting the context the parameters defined inside the with-block are also
        # available. Their keys are obtained by taking the set difference with the 
        # original parent's attribute keys and are then added to the respective
        # parameter tracking lists in the parent object.
        self._parent.MakeParameterLists(set(self._parent.__dict__.keys()) - self.keys0)
        

class TrafficArrays(object):
    """ Base class that enables vectorization of traffic data using 
        arrays and lists but also offers object-like benefits to 
        simplify adding and deleting traffic elements for all 
        parameters at once. Classes that inherit from the TrafficArrays
        class can access these features. """
    
    # The TrafficArrays class keeps track of all of the constructed
    # TrafficArrays objects (the Traffic object takes this role when it
    # is initialized because that is where aircraft are added and deleted).
    root = None

    @classmethod
    def SetRoot(cls, obj):
        """ This function is used to set the root of the tree for all 
            TrafficArrays objects."""
        cls.root = obj

    def __init__(self):
        # The TrafficArrays.root class variable is always the parent element in
        # the tree of TrafficArrays objects. All elements added after the root
        # element become children of this root element.
        self._parent   = TrafficArrays.root
        if self._parent:
            self._parent._children.append(self)

        # Keep track of child objects of the current object
        self._children = []
        
        # Parameter tracking lists for numpy array and list type variables
        self._ArrVars  = []
        self._LstVars  = []

        # Keep track of all object attributes
        self._Vars     = self.__dict__

    def reparent(self, newparent):
        """ Remove object from the parent's list of children, and add
            it to the list of children of the new parent"""

        self._parent._children.pop(self._parent._children.index(self))
        newparent._children.append(self)
        self._parent = newparent

    def MakeParameterLists(self, keys):
        """ Takes a list of keys and adds these to the object's 
             parameter tracking lists. """

        # Keys for objects of type list and numpy array are added to the respective 
        # parameter lists. If a key to an object of type TrafficArrays is passed
        # then the object is added to the current object's list of children.
        for key in keys:
            if isinstance(self._Vars[key], list):
                self._LstVars.append(key)
            elif isinstance(self._Vars[key], np.ndarray):
                self._ArrVars.append(key)
            elif isinstance(self._Vars[key], TrafficArrays):
                self._Vars[key].reparent(self)

    def create(self, n=1):
        """ Append n elements (aircraft) to all parameter lists and
            arrays. """

        for v in self._LstVars:  # Lists (mostly used for strings)

            # Get type
            vartype = None
            lst = self.__dict__.get(v)
            if len(lst) > 0:
                vartype = str(type(lst[0])).split("'")[1]

            if vartype in defaults:
                defaultvalue = [defaults[vartype]] * n
            else:
                defaultvalue = [""] * n

            self._Vars[v].extend(defaultvalue)

        for v in self._ArrVars:  # Numpy array
            # Get type without byte length
            fulltype = str(self._Vars[v].dtype)
            vartype = ""
            for c in fulltype:
                if not c.isdigit():
                    vartype = vartype + c

            # Get default value
            if vartype in defaults:
                defaultvalue = [defaults[vartype]] * n
            else:
                defaultvalue = [0.0] * n

            self._Vars[v] = np.append(self._Vars[v], defaultvalue)

    def istrafarray(self, key):
        """ Checks if key is in one of the parameter tracking lists. """
        return key in self._LstVars or key in self._ArrVars

    def create_children(self, n=1):
        """ Recursively append n elements (aircraft) to the parameter 
            lists of all child objects below the current object. """

        for child in self._children:
            child.create(n)
            child.create_children(n)

    def delete(self, idx):
        """" Recursively delete element (aircraft) idx from all lists 
             and arrays. """
        
        for child in self._children:
            child.delete(idx)

        for v in self._ArrVars:
            self._Vars[v] = np.delete(self._Vars[v], idx)

        if self._LstVars:
            if isinstance(idx, Collection):
                for i in reversed(idx):
                    for v in self._LstVars:
                        del self._Vars[v][i]
            else:
                for v in self._LstVars:
                    del self._Vars[v][idx]

    def reset(self):
        """ Recursively delete all elements from parameter lists and 
            start at 0 aircraft. Lists will become empty, arrays will
            become empty arrays of the same type. """

        for child in self._children:
            child.reset()

        for v in self._ArrVars:
            self._Vars[v] = np.array([], dtype=self._Vars[v].dtype)

        for v in self._LstVars:
            self._Vars[v] = []
