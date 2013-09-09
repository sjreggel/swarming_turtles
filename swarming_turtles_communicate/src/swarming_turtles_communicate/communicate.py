#!/usr/bin/env python
import sys
import os
import time
import xmlrpclib
import urlparse

g_publishers = {}


def make_master_uri(name):
    return 'http://%s:11311'%(name)

# Clean up the stuff that we registered
def disconnect(topic, foreign_master_uri):
    global g_publishers
    server = xmlrpclib.ServerProxy(foreign_master_uri)
    
    for p in g_publishers:
        server.unregisterPublisher(p, topic, g_publishers[p])
        print 'unregistering %s @ %s'%(p,g_publishers[p])
    g_publishers = {}

def connect(topic, foreign_master_uri):
    global g_publishers
    publishers = dict(g_publishers)

    #print publishers
    sub_uri = os.environ['ROS_MASTER_URI']
    adv_uri = foreign_master_uri

    server_sub = xmlrpclib.ServerProxy(sub_uri)
    server_adv = xmlrpclib.ServerProxy(adv_uri)

    
    code, msg, topics = server_sub.getPublishedTopics('', '')
    # Determine the type of the topic
    type = None
    for t in topics:
        if t[0] == topic:
            type = t[1]
            break
    # Find publishers
    if type is not None:
        new_publishers = {}
        code, msg, state = server_sub.getSystemState('')
        pubs = state[0]
        for t in pubs:
            if t[0] == topic:
                for p in t[1]:
                    # Hack to avoid circular re-subscription
                    dup = False
                    for pp in publishers:
                        if p.startswith(pp):
                            #print '%s is dup of %s; skipping'%(p,pp)
                            dup = True
                            break
                    if dup:
                        continue
                    code, msg, uri = server_sub.lookupNode('', p)
                    # Uniquify by master
                    sp = urlparse.urlsplit(sub_uri)
                    mangled_name = '%s_%s_%s'%(p,sp.hostname,sp.port)
                    new_publishers[mangled_name] = uri
        # Update remote advertisements appropriately
        subtractions = set(publishers.keys()) - set(new_publishers.keys())
        additions = set(new_publishers.keys()) - set(publishers.keys())
        for s in subtractions:
            server_adv.unregisterPublisher(s, topic, publishers[s])
            print 'unregistering %s @ %s'%(s,publishers[s])
        for a in additions:
            server_adv.registerPublisher(a, topic, type, new_publishers[a])
            print 'registering %s @ %s'%(a,new_publishers[a])
        publishers = new_publishers
        g_publishers = publishers


def parse(argvin):
    argv = []
    # Remove the ROS arguments.
    for arg in argvin:
        if arg.find(":=") == -1:
            argv.append(arg)
    if len(argv) != 4:
        print USAGE
        return None
    mode = argv[1]
    if mode != 'sub' and mode != 'adv':
        print USAGE
        return None
    topic = argv[2]
    foreign_master_uri = argv[3]
    return (mode, topic, foreign_master_uri)






if __name__ == '__main__':
    topic = "/test"
    foreign_master_uri = "http://tb06:11311"
    try:
        connect(topic, foreign_master_uri)
        time.sleep(4.0)
        disconnect(foreign_master_uri)
    except KeyboardInterrupt, e:
        disconnect(foreign_master_uri)

