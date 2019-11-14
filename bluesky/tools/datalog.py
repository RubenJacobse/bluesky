""" BlueSky Datalogger """

# ToDo: Add description in comments

import os
import numbers
import itertools
from datetime import datetime
import numpy as np
from bluesky import settings, stack
from bluesky.tools import varexplorer as ve
import bluesky as bs

# Register settings defaults
settings.set_variable_defaults(log_path='output')

logprecision = '%.4f'

# Dict to contain the definitions of periodic loggers
periodicloggers = dict()

# Dict to contain all loggers (also the periodic loggers)
allloggers = dict()


def crelog(name, dt=None, header=''):
    ''' Create a new logger from the stack. '''
    allloggers[name] = CSVLogger(name, dt or 0.0, header)
    if dt:
        periodicloggers[name] = allloggers[name]

    return allloggers[name]


def preupdate(simt):
    pass


def postupdate():
    """ This function writes to files of all periodic logs by calling the appropriate
    functions for each type of periodic log, at the approriate update time. """
    for log in periodicloggers.values():
        log.log()


def reset():
    """ This function closes all logs. It is called when simulation is
    reset and at quit. """

    CSVLogger.simt = 0.0

    # Close all logs and remove reference to its file object
    for log in allloggers.values():
        log.reset()


def makeLogfileName(logname):
    scenname = stack.get_scenname()
    fname = f"{logname}_{scenname}.log"
    return settings.log_path + '/' + fname, scenname


def col2txt(col, nrows):
    if isinstance(col, (list, np.ndarray)):
        if isinstance(col[0], numbers.Integral):
            ret = np.char.mod('%d', col)
        elif isinstance(col[0], numbers.Number):
            ret = np.char.mod(logprecision, col)
        else:
            ret = np.char.mod('%s', col)
        if len(ret.shape) > 1:
            for el in ret.T:
                yield el
        else:
            yield ret
    elif isinstance(col, numbers.Integral):
        yield nrows * ['%d' % col]
    elif isinstance(col, numbers.Number):
        yield nrows * [logprecision % col]
    # The input is not a number
    else:
        yield nrows * [col]


class CSVLogger:
    def __init__(self, name, dt, header):
        self.name = name
        self.file = None
        self.scenname = ""
        self.dataparents = []
        self.header = header.split('\n')
        self.tlog = 0.0
        self.selvars = []

        # In case this is a periodic logger: log timestep
        self.dt = dt
        self.default_dt = dt

        # Register a command for this logger in the stack
        stackcmd = {name: [
            name + ' ON/OFF,[dt] or ADD [FROM parent] var1,...,varn',
            '[txt,float/word,...]', self.stackio, name+" data logging on"]
        }
        stack.append_commands(stackcmd)

    def setheader(self, header):
        self.header = header.split('\n')

    def setdt(self, dt):
        self.dt = dt
        self.default_dt = dt

    def addvars(self, selection):
        selvars = []
        while selection:
            parent = ''
            if selection[0].upper() == 'FROM':
                parent = selection[1]
                del selection[0:2]
            vars = list(itertools.takewhile(
                lambda i: i.upper() != 'FROM', selection))
            selection = selection[len(vars):]
            for v in vars:
                varobj = ve.findvar(parent + '.' + v)
                if varobj:
                    selvars.append(varobj)
                else:
                    return False, 'Variable {} not found'.format(v)

        self.selvars = selvars
        return True

    def open(self, fname):
        if self.file:
            self.file.close()
        self.file = open(fname, 'wb', buffering=1)
        # Write the header
        for line in self.header:
            self.file.write(bytearray('# ' + line + '\n', 'ascii'))
        # # Write the column contents
        # columns = ['simt']
        # for v in self.selvars:
        #     columns.append(v.varname)
        # self.file.write(
        #     bytearray('# ' + str.join(', ', columns) + '\n', 'ascii'))

    def isopen(self):
        return self.file is not None

    def log(self, *additional_vars):
        if self.file and bs.sim.simt >= self.tlog:
            # Set the next log timestep
            self.tlog += self.dt

            # Make the variable reference list
            varlist = [int(bs.sim.simt)]  # NOTE cast may cause problem at dt<1
            varlist += [v.get() for v in self.selvars]
            varlist += additional_vars

            txtdata = ",".join([str(var) for var in varlist]) + "\n"

            # # Get the number of rows from the first array/list
            # nrows = 0
            # for v in varlist:
            #     if isinstance(v, (list, np.ndarray)):
            #         nrows += len(v)
            #         # break
            # # if nrows == 0:
            # #     return
            # # Convert (numeric) arrays to text, leave text arrays untouched
            # txtdata = ",".join([txtcol[0] for col in varlist
            #                     for txtcol in col2txt(col, nrows)]) + "\n"

            # log the data to file
            self.file.write(bytearray(txtdata, "ascii"))

    def start(self):
        ''' Start this logger. '''
        self.tlog = bs.sim.simt
        filepath, scenname = makeLogfileName(self.name)
        self.scenname = scenname
        self.open(filepath)

    def reset(self):
        self.dt = self.default_dt
        self.tlog = 0.0
        if self.file:
            self.file.close()
            self.file = None

    def listallvarnames(self):
        return str.join(', ', (v.varname for v in self.selvars))

    def stackio(self, *args):
        if len(args) == 0:
            text = 'This is '
            if self.name in periodicloggers:
                text += 'a periodic logger, with an update interval of %.2f seconds.\n' % self.dt
            else:
                text += 'a non-periodic logger.\n'

            text += 'with variables: ' + self.listallvarnames() + '\n'
            text += self.name + ' is ' + ('ON' if self.isopen() else 'OFF') + \
                '\nUsage: ' + self.name + \
                ' ON/OFF,[dt] or ADD [FROM parent] var1,...,varn'
            return True, text
            # TODO: add list of logging vars
        elif args[0] == 'ON':
            if len(args) > 1:
                if type(args[1]) is float:
                    self.dt = args[1]
                else:
                    return False, 'Turn ' + self.name + ' on with optional dt'
            self.start()

        elif args[0] == 'OFF':
            self.reset()

        elif args[0] == 'ADD':
            return self.addvars(list(args[1:]))

        return True
