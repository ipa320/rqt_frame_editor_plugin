#!/usr/bin/env python

## Python ##
##
import importlib

## ROS ##
##
import rospy
import actionlib

from intent_dispatcher.srv import *


class Provider:
    def __init__(self, request):
        self.provider_type_module = request.provider_type_module # e.g. "std_srvs"
        self.provider_type_name   = request.provider_type_name   # e.g. "Empty"
        self.provider_name        = request.provider_name        # e.g. "my_original_service"
        self.proxy_name           = request.proxy_name           # e.g. "my_new_service"
        self.name                 = request.name                 # human readable name
        self.description          = request.description          # human readable description
        self.icon_path            = request.icon_path


class Proxy:
    '''Every NewService object handles a callback for _one_ specific _new_ service.'''

    def __init__(self, request, chooser):

        ## Variables ##
        ##
        self.proxy_type        = request.proxy_type        # "action" or "service"
        self.proxy_type_module = request.proxy_type_module # e.g. "std_srvs"
        self.proxy_type_name   = request.proxy_type_name   # e.g. "Empty"
        self.proxy_name        = request.proxy_name        # e.g. "my_new_service"
        self.name              = request.name              # human readable name
        self.description       = request.description       # human readable description
        self.icon_path         = request.icon_path
    
        self.providers = {}

        self.choose_provider = chooser
        self.active = False

        ## Module name ##
        ##
        if self.proxy_type == "action":
            module_name = self.proxy_type_module + ".msg"
        elif self.proxy_type == "service":
            module_name = self.proxy_type_module + ".srv"
        else:
            print "Proxy type has to be 'action' or 'service'"
            raise ValueError


        ## Import module ##
        ##
        imported_module = importlib.import_module(module_name) # raises ImportError
        self.proxy_class = getattr(imported_module, self.proxy_type_name)

        ## Provide action/service ##
        ##
        if self.proxy_type == "action":
            self.action_server = actionlib.SimpleActionServer(self.proxy_name, self.proxy_class, execute_cb=self.execute_callback, auto_start=False)
            self.action_server.start()

        elif self.proxy_type == "service":
            self.service = rospy.Service(self.proxy_name, self.proxy_class, self.service_callback)


    def add_provider(self, provider_name, provider):
        print "Adding provider: ", provider_name
        self.providers[provider_name] = provider
        print self.providers
    

    ## SERVICES ##
    ##
    def service_callback(self, req):
        print "Handling callback!"
        
        ## Look for recipient ##
        ##
        provider = self.choose_provider(self, self.providers)

        ## Call service ##
        ##
        print "Calling service"
        service = rospy.ServiceProxy(provider.provider_name, self.proxy_class)
        return service(req)


    ## ACTIONS ##
    ##    
    def execute_callback(self, goal):
        print "Handling callback!"

        ## Look for recipient ##
        ##
        provider = self.choose_provider(self, self.providers)

        ## Call action ##
        ##
        print "Calling action"
        client = actionlib.SimpleActionClient(provider.provider_name, self.proxy_class)
        client.wait_for_server()
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        result = client.wait_for_result()
        self.action_server.set_succeeded(result)
        print "result done"

    def feedback_callback(self, feedback):
        self.action_server.publish_feedback(feedback)
        

def text_chooser(providers):
    if len(providers) == 0:
        return None
    else:
        print "Which action should be taken?"
        for i, provider in enumerate(providers):
            print i,":", provider

        ## Let user decide ##
        ##
        names = providers.keys()
        try:
            num = int(raw_input('Number: '))
            return providers[names[num]]
        except:
            print "Not a number or number not valid. Failed. Returning!"
            return None ## TODO
        
   
class Dispatcher:

    def __init__(self):
        self.proxies = {}

        ## Register service ##
        ##
        rospy.Service("register_proxy",       AddProxy, self.add_proxy_callback)
        rospy.Service("register_provider", AddProvider, self.add_provider_callback)

        self.set_chooser(text_chooser)


    def set_chooser(self, chooser):
        self.default_chooser = chooser

    
    def add_proxy_callback(self, req):

        ## Check ##
        ##
        if req.proxy_name in self.proxies:
            print "Proxy already exists:", self.proxies[req.proxy_name]
            return AddProxyResponse()

        ## Create Proxy ##
        ##
        self.proxies[req.proxy_name] = Proxy(req, self.default_chooser)
        
        return AddProxyResponse()


    def add_provider_callback(self, req):

        ## Check ##
        ##
        if not req.proxy_name in self.proxies:
            print "Proxy does not exist", req.proxy_name
            return AddProxyResponse()

        ## Add provider ##
        ##
        provider = Provider(req)
        self.proxies[req.proxy_name].add_provider(req.provider_name, provider)
        
        return AddProviderResponse()
   
    
if __name__ == "__main__":

    rospy.init_node('intent_dispatcher')

    dispatcher = Dispatcher()

    print "Dispatcher ready: Call service '/register_intent' to register a new service."
    rospy.spin()
    