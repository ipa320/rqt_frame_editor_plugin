#!/usr/bin/env python

import os
import xmlrpclib

import rospy
import rosservice


class ProviderLookup():

    def __init__(self, skilltopic):
        self._master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])


    def lookup_providers(self, no_loggers=True):

        ## System State ##
        ##
        code, msg, val = self._master.getSystemState("/ProviderLookup")
        if code != 1:
            rospy.logerr("Call to rosmaster failed: %s, %s", code, msg)
            return [], []
        pubs, subs, srvs = val
        

        code, msg, topics = self._master.getPublishedTopics("/Collector", '/')
        if code != 1:
            rospy.logerr("Call to rosmaster failed: %s, %s", code, msg)
            return [], []

        ## List of all published topics
        publishers = []
        for publisher in topics:
            publishers.append(publisher[0])



        ## SERVICES ##
        ##
        services = []
        for service in srvs:
            # (service name, service type, node name)
            if no_loggers:
                if service[0].endswith("/get_loggers") or service[0].endswith("/set_logger_level"):
                    continue

            services.append( (service[0], rosservice.get_service_type(service[0]), service[1][0]) )


        ## ACTIONS ##
        ##
        actions = []
        for sub in subs:
            # Check all subscribers with '/goal' at the end 
            # (result, feedback and status must exist for actions)
            if sub[0].endswith('/goal'):
                action_name = sub[0]
                action_name = action_name[:-5] # cut off 'goal'

                ## Check publishers
                if action_name+'/feedback' not in publishers:
                    print "narf1"
                    continue
                if action_name+'/status' not in publishers:
                    print "narf2"
                    continue
                if action_name+'/result' not in publishers:
                    print "narf3"
                    continue

                ## Get type
                action_type = None
                for top in topics:
                    if top[0] == action_name+'/result':
                        action_type = top[1][:-6] # cut off '/result'
                        break

                if action_type is None:
                    continue

                # (action name, action type, node name)
                actions.append( (action_name, action_type, sub[1][0]) )

        return services, actions

 
if __name__ == "__main__":
    try:
        rospy.init_node('provider_lookup')

        rospy.loginfo("Looking for provided services and actions")

        pl = ProviderLookup('testTopic')
        services, actions = pl.lookup_providers()

        rospy.loginfo("Services:\n %s", str(services))
        rospy.loginfo("Actions:\n %s", str(actions))

    except Exception as e:
        print e

# eof
