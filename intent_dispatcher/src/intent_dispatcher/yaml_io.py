#!/usr/bin/python

## Python ##
##

## ROS ##
##
import rospy
import rosparam

from intent_dispatcher import dispatcher


def export_yaml(disp, filename, namespace = "intent_dispatcher/"):

    proxy_data = {}
    provider_data = []
    
    ## Go through proxies and their providers
    for proxy in disp.proxies.values():

        ## Providers        
        for provider in proxy.providers.values():
            provider_data.append({
                'provider_type_module' : provider.provider_type_module, # e.g. "std_srvs"
                'provider_type_name'   : provider.provider_type_name,   # e.g. "Empty"
                'provider_name'        : provider.provider_name,        # e.g. "my_original_service"
                'proxy_name'           : provider.proxy_name,           # e.g. "my_new_service"
                'name'                 : provider.name,                 # human readable name
                'description'          : provider.description,          # human readable description
                'icon_path'            : provider.icon_path
            })

        ## Proxy
        proxy_data[proxy.proxy_name] = {
            'proxy_type'        : proxy.proxy_type,        # "action" or "service"
            'proxy_type_module' : proxy.proxy_type_module, # e.g. "std_srvs"
            'proxy_type_name'   : proxy.proxy_type_name,   # e.g. "Empty"
            'proxy_name'        : proxy.proxy_name,        # e.g. "my_new_service"
            'name'              : proxy.name,              # human readable name
            'description'       : proxy.description,       # human readable description
            'icon_path'         : proxy.icon_path,
        }

    data = {}
    data["proxies"] = proxy_data
    data["providers"] = provider_data


    ## To parameter server
    rospy.set_param(namespace, data)
    print rospy.get_param(namespace)


    ## Dump param to file
    print "Saving to file", filename
    rosparam.dump_params(filename, namespace)
    print "Saving done"




def import_yaml(disp, filename, namespace = "intent_dispatcher/"):
    print "Loading file"

    ## Load yaml file
    data = rosparam.load_file(filename, namespace)[0][0]

    ## Import proxies
    for proxy in data["proxies"].values():
        request = dispatcher.Proxy_Request(**proxy)
        disp.add_proxy_callback(request)

    ## Import providers
    for provider in data['providers']:
        request = dispatcher.Provider_Request(**provider)
        disp.add_provider_callback(request)

    print "Loading done"
