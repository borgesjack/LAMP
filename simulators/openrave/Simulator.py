import os
import sys

import openravepy

# this class is the origin for the openRAVE simulator stuff
class Simulator(object):
    def __getattr__(self,name):
        if hasattr(openravepy,name):
            return getattr(openravepy,name)
        else:
            raise AttributeError("{} attribute not found".format(name))
