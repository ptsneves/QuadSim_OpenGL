#!/usr/bin/env python
'''
Fork from Ardupilot Simulator
@author: pneves
'''
# Basic OBJ file viewer. needs objloader from:
#  http://www.pygame.org/wiki/OBJFileLoader
# LMB + move: rotate
# RMB + move: pan
# Scroll wheel: zoom in/out
import sys, pygame, signal
import gst
import os
from pygame.locals import *
from pygame.constants import *
from OpenGL.GL import *
from OpenGL.GLU import *
#from ctypes import *
#from OpenGL.arrays import ArrayDatatype as ADT
from OpenGL.arrays import vbo
import numpy 
# IMPORT OBJECT LOADER
from ObjLoader import *
import os
from ctypes import c_float

from multicopter import MultiCopter
import util, time, os, sys, math
import socket, struct
import select, errno
import threading
import thread

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..', '..', 'mavlink', 'pymavlink'))

current_time = None

shutdown_flag = False

#http://stackoverflow.com/questions/19988726/why-is-signal-sigterm-not-dealt-with-properly-in-my-main-thread?rq=1
def sighandler(signum, frame):
    print 'signal handler called with signal: %s ' % signum
    global shutdown_flag
    shutdown_flag = True
    sys.exit() # make sure you add this so the main thread exits as well.

def sim_send(m, multicopter_object):
    '''send flight information to mavproxy and flightgear'''
    global fdm
    from math import degrees

    earth_rates = util.BodyRatesToEarthRates(multicopter_object.dcm, multicopter_object.gyro)
    (roll, pitch, yaw) = multicopter_object.dcm.to_euler()


    buf = struct.pack('<17dI',
                      multicopter_object.latitude, multicopter_object.longitude, multicopter_object.altitude, degrees(yaw),
                      multicopter_object.velocity.x, multicopter_object.velocity.y, multicopter_object.velocity.z,
                      multicopter_object.accelerometer.x, multicopter_object.accelerometer.y, multicopter_object.accelerometer.z,
                      degrees(earth_rates.x), degrees(earth_rates.y), degrees(earth_rates.z),
                      degrees(roll), degrees(pitch), degrees(yaw),
                      math.sqrt(multicopter_object.velocity.x*multicopter_object.velocity.x + multicopter_object.velocity.y*multicopter_object.velocity.y),
                      0x4c56414f)
    try:
        sim_out.send(buf)
    except socket.error as e:
        if not e.errno in [ errno.ECONNREFUSED ]:
            raise


def sim_recv(m):
    '''receive control information from SITL'''
    try:
        buf = sim_in.recv(28)
    except socket.error as e:
        if not e.errno in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
            raise
        return
        
    if len(buf) != 28:
        return
    control = list(struct.unpack('<14H', buf))
    pwm = control[0:11]

    # update motors
    for i in range(11):
        m[i] = (pwm[i]-1000)/1000.0

    # update wind
    global multicopter_object
    (speed, direction, turbulance) = control[11:]
    multicopter_object.wind.speed = speed*0.01
    multicopter_object.wind.direction = direction*0.01
    multicopter_object.wind.turbulance = turbulance*0.01
    


def interpret_address(addrstr):
    '''interpret multicopter_object IP:port string'''
    multicopter_object = addrstr.split(':')
    multicopter_object[1] = int(multicopter_object[1])
    return tuple(multicopter_object)

def render_sim(resolution, fov, altitude, platform_image, render_only = False):
    dred_lock = threading.Lock()#lol
    
    pygame.init()
    viewport = resolution
    hx = viewport[0]/2
    hy = viewport[1]/2
    srf = pygame.display.set_mode(viewport, OPENGL | DOUBLEBUF)
    gst.parse_launch("ximagesrc xname=\"pygame window\" ! ffmpegcolorspace ! video/x-raw-rgb,framerate=10/1 ! v4l2sink device=/dev/video0").set_state(gst.STATE_PLAYING)
    glLightfv(GL_LIGHT0, GL_POSITION,  (-40, 200, 100, 0.0))
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.5, 0.5, 0.5, 1.0))
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHTING)
    glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)           # most obj files expect to be smooth-shaded
     
    # LOAD OBJECT AFTER PYGAME INIT
    dir_path = os.path.dirname(os.path.abspath(__file__))
    objfile = dir_path + "/Platform_Support.obj"
    obj = OBJ(objfile, platform_image, swapyz=True)
    
    #http://stackoverflow.com/questions/13179565/how-to-get-vbos-to-work-with-python-and-pyopengl
    #http://stackoverflow.com/questions/8259628/triangle-texture-mapping-opengl
    
    
    
    clock = pygame.time.Clock()
    #http://www.bobatkins.com/photography/technical/measuring_focal_length.html
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    width, height = viewport
    
    #http://forums.logitech.com/t5/Webcams/Webcam-c500-fov-and-focal-length/td-p/699626
    gluPerspective(fov, width/float(height), 0.01, 100.0)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_MODELVIEW)
    
    if render_only is True:
        rx, ry = (0,0)
        tx, ty = (0,0)
        zpos = altitude
        rotate = move = False
        clock = pygame.time.Clock()
	
	
    while shutdown_flag is False:
     	current_time = time.time()    
        if render_only is False:
            global frame_count
            global lastt
            local_multicopter_object = None
            local_current_time = None
            with dred_lock:
                local_multicopter_object = multicopter_object
                local_current_time = current_time
#         if local_current_time == 0 or local_multicopter_object == None:
#             continue
         
#         print("%.2f fps sleepOverhead=%f zspeed=%.2f zaccel=%.2f h=%.1f local_multicopter_object=%.1f yaw=%.1f" % (
#             frame_count/(local_current_time-lastt),
#             sleep_overhead,
#             local_multicopter_object.velocity.z, local_multicopter_object.accelerometer.z, local_multicopter_object.position.z, local_multicopter_object.altitude,
#             local_multicopter_object.yaw))
#         lastt = local_current_time
#         frame_count = 0
        #http://stackoverflow.com/questions/2683205/pyopengl-glvertexpointer-offset-problem 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        if render_only is False:
            (roll, pitch, yaw) = local_multicopter_object.dcm.to_euler() #in radians
            #print local_multicopter_object.position.x, local_multicopter_object.position.y, local_multicopter_object.position.z
            # RENDER OBJECT
            
            glTranslate(-local_multicopter_object.position.y, local_multicopter_object.position.x, - local_multicopter_object.altitude)
            glRotate(-math.degrees(roll), 0, 1, 0)
            glRotate(math.degrees(pitch), 1, 0, 0)
            glRotate(math.degrees(yaw), 0, 0, 1)
            #print local_multicopter_object.position.x, local_multicopter_object.position.y, - local_multicopter_object.altitude
            #print math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
        else:
            clock.tick(30)
            for e in pygame.event.get():
                if e.type == QUIT:
                    sys.exit()
                elif e.type == KEYDOWN and e.key == K_ESCAPE:
                    sys.exit()
                elif e.type == MOUSEBUTTONDOWN:
                    if e.button == 4: zpos = max(1, zpos-1)
                    elif e.button == 5: zpos += 1
                    elif e.button == 1: rotate = True
                    elif e.button == 3: move = True
                elif e.type == MOUSEBUTTONUP:
                    if e.button == 1: rotate = False
                    elif e.button == 3: move = False
                elif e.type == MOUSEMOTION:
                    i, j = e.rel
                    #if rotate:
                        #rx += i
                        #ry += j
                    if move:
                        tx += i
                        ty -= j
         
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
         
            # RENDER OBJECT
            glTranslate(tx/20., ty/20., - zpos)
            glRotate(-ry, 1, 0, 0)
            glRotate(-rx, 0, 1, 0)
            glRotate(90, 0, 0, 1)
        glCallList(obj.gl_list)
        pygame.display.flip()
        frame_end = time.time()
        time_left = 0.04 - (frame_end - current_time)
        if time_left > 0:
        	time.sleep(time_left)
        	
##################
# main program
from optparse import OptionParser
parser = OptionParser("sim_multicopter.py [options]")
parser.add_option("--simin",  dest="simin",   help="SIM input (IP:port)",       default="127.0.0.1:5502")
parser.add_option("--simout", dest="simout",  help="SIM output (IP:port)",      default="127.0.0.1:5501")
parser.add_option("--fgout", dest="fgout",  help="flightgear output (IP:port)", default="127.0.0.1:5503")
parser.add_option("--home", dest="home",  type='string', default=None, help="home lat,lng,alt,hdg")
parser.add_option("--rate", dest="rate", type='int', help="SIM update rate", default=200)
parser.add_option("--wind", dest="wind", help="Simulate wind (speed,direction,turbulance)", default='0,0,0')
parser.add_option("--frame", dest="frame", help="frame type (+,X,octo)", default='+')
parser.add_option("--fov", dest="fov_arg", help="field of view in radiwans", default=None)
parser.add_option("--resolution", dest="resolution_arg", help="Resolutin in pixels", default=None)
parser.add_option("--render", dest="render_arg", help="Resolutin in pixels", default=False)
parser.add_option("--image", dest="platform_image_arg", help="Platform Texture", default="OBJ")
(opts, args) = parser.parse_args()

render = False
for m in [ 'home', 'fov_arg', 'resolution_arg' ]:
    if not opts.__dict__[m]:
        print("Missing required option '%s'" % m)

resolution = opts.resolution_arg.split(",")
resolution[0] = int(resolution[0])
resolution[1] = int(resolution[1])
if len(resolution) != 2:
    print("Resolution should be X,Y")
    sys.exit(1)


# parse home
v = opts.home.split(',')
if len(v) != 4:
    print("home should be lat,lng,alt,hdg")
    sys.exit(1)
    
# create the quadcopter model
multicopter_object = MultiCopter(frame=opts.frame)
    
multicopter_object.home_latitude = float(v[0])
multicopter_object.home_longitude = float(v[1])
multicopter_object.home_altitude = float(v[2]) / 1000
multicopter_object.latitude = multicopter_object.home_latitude
multicopter_object.longitude = multicopter_object.home_longitude
multicopter_object.altitude = multicopter_object.home_altitude
multicopter_object.yaw = float(v[3])
multicopter_object.ground_level = multicopter_object.home_altitude
multicopter_object.position.z = 0
multicopter_object.wind = util.Wind(opts.wind)


if bool(opts.render_arg) is True:
    render_sim(resolution, math.degrees(float(opts.fov_arg)),
     multicopter_object.home_altitude, opts.platform_image_arg, True )
    
signal.signal(signal.SIGTERM, sighandler)
signal.signal(signal.SIGABRT, sighandler)    
    
rame_time = 1.0/opts.rate
sleep_overhead = 0
# UDP socket addresses
fg_out_address  = interpret_address(opts.fgout)
sim_out_address = interpret_address(opts.simout)
sim_in_address  = interpret_address(opts.simin)

# setup output to flightgear
fg_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fg_out.connect(fg_out_address)
fg_out.setblocking(0)

# setup input from SITL
sim_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_in.bind(sim_in_address)
sim_in.setblocking(0)

# setup output to SITL
sim_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sim_out.connect(sim_out_address)
sim_out.setblocking(0)



("Simulating %u motors for frame %s" % (len(multicopter_object.motors), opts.frame))

# motors initially off
m = [0.0] * 11

lastt = time.time()
frame_count = 0

print("Starting at lat=%f lon=%f alt=%.1f heading=%.1f" % (
    multicopter_object.home_latitude,
    multicopter_object.home_longitude,
    multicopter_object.home_altitude,
    multicopter_object.yaw))


frame_time = 1.0/opts.rate
sleep_overhead = 0

thread.start_new_thread(render_sim,(resolution, math.degrees(float(opts.fov_arg)), \
	multicopter_object.home_altitude, opts.platform_image_arg) )
while shutdown_flag is False:
    frame_start = time.time()
    sim_recv(m)

    m2 = m[:]

    multicopter_object.update(m2)
    sim_send(m, multicopter_object)
    frame_count += 1
    current_time = time.time()
    
    frame_end = time.time()
    if frame_end - frame_start < frame_time:
        dt = frame_time - (frame_end - frame_start)
        dt -= sleep_overhead
        if dt > 0:
            time.sleep(dt)
        sleep_overhead = 0.99*sleep_overhead + 0.01*(time.time() - frame_end - dt)
