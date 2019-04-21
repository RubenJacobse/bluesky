# -*- coding: utf-8 -*-
"""
Classes that derive from TrafficArrays (like Traffic) get automated create,
delete, and reset functionality for all registered child arrays.
"""

try:
    from collections.abc import Collection
except ImportError:
    # In python <3.3 collections.abc doesn't exist
    from collections import Collection
import numpy as np

VAR_DEFAULTS = {"float": 0.0,
                "int": 0,
                "uint": 0,
                "bool": False,
                "S": "",
                "str": ""}


class RegisterElementParameters():
    """
    Class to use in 'with'-syntax. This class automatically calls for
    the make_parameter_lists function of the TrafficArrays object inside which
    it is called, and registers the parameters defined in the 'with' block.
    """

    def __init__(self, parent):
        # On initialization store the parent object as an attribute
        self._parent = parent

    def __enter__(self):
        # On entering the context only the parent's attributes are present,
        # store the keys of these attributes as a set().
        self.keys_initial = set(self._parent.__dict__.keys())

    def __exit__(self, type, value, tb):
        # On exiting the context the parameters defined inside the with-block are also
        # available. Their keys are obtained by taking the set difference with the
        # original parent's attribute keys and are then added to the respective
        # parameter tracking lists in the parent object.
        keys_current = set(self._parent.__dict__.keys())
        self._parent.make_parameter_lists(keys_current - self.keys_initial)


class TrafficArrays(object):
    """
    Base class that enables vectorization of traffic data using arrays
    and lists but also offers object-like benefits to simplify adding
    and deleting traffic elements for all parameters at once.
    """

    # The TrafficArrays class keeps track of all of the constructed
    # TrafficArrays objects (the Traffic object becomes the 'root' when it
    # is initialized because that is where aircraft are added and deleted).
    root = None

    @classmethod
    def set_class_root(cls, obj):
        """
        This function is used to set the root element of the tree of all
        TrafficArrays objects.
        """

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

    def reparent(self, new_parent):
        """
        Remove current object from the parent's list of children, and add
        it to the list of children of the new parent.
        """

        self._parent._children.pop(self._parent._children.index(self))
        new_parent._children.append(self)
        self._parent = new_parent

    def make_parameter_lists(self, keys):
        """
        Takes a list of parameter keys and adds these to the object's
        parameter tracking lists.
        """

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
        """
        Append n elements (aircraft) to all parameter lists and arrays.
        """

        for var in self._LstVars:  # Lists (mostly used for strings)

            # Get type
            var_type = None
            lst = self.__dict__.get(var)
            if len(lst) > 0:
                var_type = str(type(lst[0])).split("'")[1]

            if var_type in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[var_type]] * n
            else:
                defaultvalue = [""] * n

            self._Vars[var].extend(defaultvalue)

        for var in self._ArrVars:  # Numpy array
            # Get type without byte length
            var_type = ''.join(c for c in str(self._Vars[var].dtype) if c.isalpha())

            # Get default value
            if var_type in VAR_DEFAULTS:
                defaultvalue = [VAR_DEFAULTS[var_type]] * n
            else:
                defaultvalue = [0.0] * n

            self._Vars[var] = np.append(self._Vars[var], defaultvalue)

    def is_traf_array(self, key):
        """  Checks if key is in one of the parameter tracking lists. """

        return key in self._LstVars or key in self._ArrVars

    def create_children(self, n=1):
        """
        Recursively append n elements (aircraft) to the parameter lists
        of all child objects below the current object.
        """

        for child in self._children:
            child.create(n)
            child.create_children(n)

    def delete(self, idx):
        """"
        Recursively delete element (aircraft) idx from all lists and arrays.
        """

        for child in self._children:
            child.delete(idx)

        for var in self._ArrVars:
            self._Vars[var] = np.delete(self._Vars[var], idx)

        if self._LstVars:
            if isinstance(idx, Collection):
                for i in reversed(idx):
                    for var in self._LstVars:
                        del self._Vars[var][i]
            else:
                for var in self._LstVars:
                    del self._Vars[var][idx]

    def reset(self):
        """
        Recursively delete all elements from parameter lists and start at
        zero aircraft. Lists will be emptied, numpy arrays will become empty
        arrays with the same dtype.
        """

        for child in self._children:
            child.reset()

        for var in self._ArrVars:
            self._Vars[var] = np.array([], dtype=self._Vars[var].dtype)

        for var in self._LstVars:
            self._Vars[var] = []
