#!/usr/bin/env python

from intent_dispatcher.srv import *


def Provider_Request(provider_type_module, provider_type_name, provider_name, proxy_name, name, description, icon_path):
    request = AddProvider()

    request.provider_type_module = provider_type_module
    request.provider_type_name = provider_type_name
    request.provider_name = provider_name
    request.proxy_name = proxy_name
    request.name = name
    request.description = description
    request.icon_path = icon_path

    return request


class Provider:
    def __init__(self, request):
        self.provider_type_module = request.provider_type_module # e.g. "std_srvs"
        self.provider_type_name   = request.provider_type_name   # e.g. "Empty"
        self.provider_name        = request.provider_name        # e.g. "my_original_service"
        self.proxy_name           = request.proxy_name           # e.g. "my_new_service"
        self.name                 = request.name                 # human readable name
        self.description          = request.description          # human readable description
        self.icon_path            = request.icon_path


# eof
