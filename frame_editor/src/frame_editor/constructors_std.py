#!/usr/bin/env python

from std_msgs.msg import ColorRGBA, Header


## Color ##
##
def NewColor(r, g, b, a=1.0):
    col = ColorRGBA()
    col.r = r
    col.g = g
    col.b = b
    col.a = a
    return col

# eof
