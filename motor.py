#!/usr/bin/env python3
#
# Author: Andrew Maule
# Objective: - Run and record motor vibrations by looping through a set of PWM values to drive a brushless DC motor.
#            - Download corresponding PX4 log
#
#

import sys
assert sys.version_info >= (3,4)

from concurrent.futures import *
import copy
from datetime import datetime
import glob
import io
import json
from optparse import OptionParser
import os
from pymavlink import mavutil, mavparm
try:
    import queue as Queue
except ImportError:
    import Queue
import requests
import select
import signal
import subprocess as sp
import struct
from threading import Lock, Timer, Thread
from time import ctime, time, sleep

UINT16_MAX = 65535

mav_motor_messages = {
                        "VIBRATION": "mav_vibration",
                        "LOG_ENTRY": "mav_handle_log_entry", 
                        "LOG_DATA": "mav_handle_log_data"
                     }

def parse_args():
    parser = OptionParser("{} [options]".format(sys.argv[0]))

    parser.add_option("--master", dest="master", default="udpin:0.0.0.0:14551",
                      metavar="DEVICE", help="MAVLink master port and optional baud rate")
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=57600)
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=253, help='The source system identifier for this script that sends commands to the PX4 autopilot.')
    parser.add_option("--source-component", dest='SOURCE_COMPONENT', type='int', 
                      default=0, help='MAVLink source component identifier for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=1, help='Target system identifier for PX4 autopilot.')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=0, help='Target component identifier for PX4 autopilot.')
    parser.add_option("--mav10", action='store_true', default=False, help="Use MAVLink protocol 1.0")
    parser.add_option("--mav20", action='store_true', default=True, help="Use MAVLink protocol 2.0")
    parser.add_option("--version", action='store_true', help="version information")
    parser.add_option("--dialect",  default="common", help="MAVLink dialect")
    parser.add_option("--rtscts", action='store_true', help="enable hardware RTS/CTS flow control")
    parser.add_option("--input", help="Input file containing sequence of PWM values (duty cycle in nanoseconds) to loop through (newline separated values).")
    parser.add_option("--pwmchip", default="/sys/class/pwm/pwmchip0", help="Linux sysfs path to PWM chip.")
    parser.add_option("--pwmchan", type='int', default=0, help="PWM channel index to drive PWM signal output.")
    parser.add_option("--pwmperiod", type='int', default=2500000, help="The period of the PWM signal that drives the motor ESC (nanoseconds).")
    parser.add_option("--runtime", type='int', default=30, help="The number of seconds to drive the motors at each PWM value in the input file.")
    parser.add_option("--motor", help="Name of motor we are running tests against.")
    parser.add_option("-d", "--download-logs", dest="download_logs", action='store_true', help="Download the px4 log for each pwm iteration (NOTE: Only recommended for high-speed datalinks)")
    parser.add_option("--log", help="Mavlink log file.")
    return(parser.parse_args())


class MotorMavLink:
    def __init__(self, mav10, mav20, source_system, source_component, target_system, target_component, rtscts, baudrate, descriptor, log_file=None):
        self.source_system      = source_system
        self.source_component   = source_component
        self.target_system      = target_system
        self.target_component   = target_component
        self.descriptor         = descriptor 
        self.baudrate           = baudrate
        self.rtscts             = rtscts
        self.flushlogs          = True
        self.log_exit           = False
        self.vibration_log_fd   = None
        self.download_file      = None
        self.download_ofs       = 0
        self.logq               = None
        self.log_file           = None
        if( log_file ):
            self.logq           = Queue.Queue()
            self.logfd          = open(log_file, 'wb')
            self.logq.queue.clear() #Flush the queue before starting
            self.logt           = Thread(target=self.logqt, name='logqt: {}'.format(self))
            self.logt.daemon    = True
            self.logt.start()
        self.recvq              = Queue.Queue() #Mavlink receive/process message queue
        self.recvq_exit         = False
        self.recvt              = Thread(target=self.recvq_process, name='recvq_process: {}'.format(self))
        self.recvt.daemon       = True
        self.recvt.start()
        self.link_add(descriptor)
        self.heartbeat_period   = mavutil.periodic_event(0.5)
        self.rc_period          = mavutil.periodic_event(0.5)
        self.rc_outputs         = [1500, 1500, 1500, 1500, 1400]
        self.px4_log_reset()
        self.runtime_exit       = False
        self.runt = Thread(target=self.run_thread, name='run_thread: {}'.format(self))
        self.runt.daemon = True
        self.runt.start()

    def __del__(self):
        #If we are actively downloading a log file, then send a mavlink message to remote to cancel operation (so that it doesn't keep pushing data)
        if self.download_file:
            if self.master:
                self.master.mav.log_request_end_send(self.target_system,
                                                    self.target_component)
        self.runtime_exit = True
        if self.logq:
            self.log_exit = True
            if self.logqt:
                self.logqt.join(1.0)
        if self.recvt:
            self.recvq_exit = True
            self.recvt.join(1.0)
        if self.runt:
            self.runt.join(1.0)
        self.logqt = None
        self.runt = None
        self.recvt = None
        return

    def px4_log_reset(self):
        self.download_set = set()
        self.download_file = None
        self.download_lognum = None
        self.download_filename = None
        self.download_start = None
        self.download_last_timestamp = None
        self.download_ofs = 0
        self.download_last_log_num = -1
        self.retries = 0
        self.entries = {}
        self.download_queue = []

    def logqt(self):
        '''log queue thread'''
        while not self.log_exit:
            timeout = time() + 10
            while not self.logq.empty() and time() < timeout:
                msg = self.logq.get()
                #Do a sanity check to make sure the message at offset 8 is the mavlink start byte
                self.logfd.write(msg)
            if self.flushlogs or time() >= timeout:
                self.logfd.flush()

    def master_send_callback(self, m, master):
        '''called on sending a message'''
        if( self.logq ):
            sysid = m.get_srcSystem()
            mtype = m.get_type()
            message = copy.deepcopy(m.get_msgbuf())
            self.logq.put(message)
        return

    def start_vibration_log(self, logfile):
        if self.vibration_log_fd:
            fd = self.vibration_log_fd
            self.vibration_log_fd = None
            fd.close()
        self.vibration_log_fd = open(logfile, 'w')

    def stop_vibration_log(self):
        if self.vibration_log_fd:
            fd = self.vibration_log_fd
            self.vibration_log_fd = None
            fd.close()

    def read(self, timeout):
        master = self.master
        rin = []
        if master.fd is not None and not master.portdead:
            rin.append(master.fd)
        if rin != []:
            try:
                (rin, win, xin) = select.select(rin, [], [], timeout)
            except select.error:
                rin = []
                pass 
        for fd in rin:
            if fd == master.fd:
                process_master(master, self)
        return

    def send_heartbeat(self):
        if self.master.mavlink10():
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
    def send_rc_override(self):
        if self.master.mavlink10():
            self.master.mav.rc_channels_override_send(self.target_system, self.target_component, self.rc_outputs[0], self.rc_outputs[1], self.rc_outputs[2], self.rc_outputs[3], self.rc_outputs[4], 1000, 1000, 1000)

    def periodic_triggers(self):
        if( self.heartbeat_period.trigger() ):
            self.send_heartbeat()
        if( self.rc_period.trigger() ):
            self.send_rc_override()

    def parse_link_attributes(self, some_json):
        '''return a dict based on some_json (empty if json invalid)'''
        try:
            return json.loads(some_json)
        except ValueError:
            print('Invalid JSON argument: {0}'.format(some_json))
        return {}

    def parse_link_descriptor(self, descriptor):
        '''parse e.g. 'udpin:127.0.0.1:9877:{"foo":"bar"}' into
        python structure ("udpin:127.0.0.1:9877", {"foo":"bar"})'''
        optional_attributes = {}
        link_components = descriptor.split(":{", 1)
        device = link_components[0]
        if (len(link_components) == 2 and link_components[1].endswith("}")):
            # assume json
            some_json = "{" + link_components[1]
            optional_attributes = self.parse_link_attributes(some_json)
        return (device, optional_attributes)

    def apply_link_attributes(self, conn, optional_attributes):
        for attr in optional_attributes:
            print("Applying attribute to link: %s = %s" % (attr, optional_attributes[attr]))
            setattr(conn, attr, optional_attributes[attr])

    def link_add(self, descriptor, force_connected=False):
        '''add new link'''
        try:
            (device, optional_attributes) = self.parse_link_descriptor(descriptor)
            print("Connect %s source_system=%d" % (device, self.source_system))
            try:
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                    source_system=self.source_system,
                                                    baud=self.baudrate,
                                                    force_connected=force_connected)
            except Exception as e:
                # try the same thing but without force-connected for
                # backwards-compatability
                conn = mavutil.mavlink_connection(device, autoreconnect=True,
                                                    source_system=self.source_system,
                                                    baud=self.baudrate)
            conn.mav.srcComponent = self.source_component
        except Exception as msg:
            print("Failed to connect to %s : %s" % (descriptor, msg))
            return False
        if self.rtscts:
            conn.set_rtscts(True)
        conn.mav.set_callback(self.master_callback, conn)
        if hasattr(conn.mav, 'set_send_callback'):
            conn.mav.set_send_callback(self.master_send_callback, conn)
        conn.linknum = 1
        conn.linkerror = False
        conn.link_delayed = False
        conn.last_heartbeat = 0
        conn.last_message = 0
        conn.highest_msec = 0
        conn.target_system = self.target_system
        self.apply_link_attributes(conn, optional_attributes)
        self.master = conn
        return True

    def master_msg_handling(self, m, master):
        '''link message handling for an upstream link'''
        if self.target_system != 0 and master.target_system != self.target_system:
            # keep the pymavlink level target system aligned with the MAVProxy setting
            print("change target_system %u" % self.target_system)
            master.target_system = self.target_system

        if self.target_component != mavutil.mavlink.MAV_COMP_ID_ALL and master.target_component != self.target_component:
            # keep the pymavlink level target component aligned with the MAVProxy setting
            print("change target_component %u" % self.target_component)
            master.target_component = self.target_component

        #Any messages not coming from our targeted system (GCS) will be ignored
        if (self.target_system == 0) or (m.get_srcSystem() == self.target_system):
            mtype = m.get_type()
            if( mtype in mav_motor_messages ):
                decoded_message_dict = self.mav_decode(m, master)
                #If the target_system and target_component are embedded in a message, then check that they are valid for this component
                self.recvq.put((getattr(self, mav_motor_messages[mtype]), m, master, decoded_message_dict))

    def recvq_process(self):
        while not self.recvq_exit:
            (cb, m, master, decoded_message_dict) = self.recvq.get()
            cb(m, master, decoded_message_dict) #Make a call to process appropriate message processing callback
        return

    def master_callback(self, m, master):
        '''process mavlink message m on master, sending any messages to recipients'''
        # see if it is handled by a specialised sysid connection
        sysid = m.get_srcSystem()
        compid = m.get_srcComponent()
        mtype = m.get_type()
        
        if( self.logq ):
            message = copy.deepcopy(m.get_msgbuf())
            self.logq.put(message)

        if getattr(m, '_timestamp', None) is None:
            master.post_message(m)

        self.master_msg_handling(m, master)

    def mav_decode(self, m, master):
        b = m.get_msgbuf()
        dm = master.mav.decode(b)
        dmd = dm.to_dict()
        return(dmd)

    def mav_vibration(self, m, master, dmd):
        vibration_keys = ['time_usec', 'vibration_x', 'vibration_y', 'vibration_z', 'clipping_0', 'clipping_1', 'clipping_2']
        if( self.vibration_log_fd ):
            vibration_data = [str(dmd[k]) for k in vibration_keys]
            self.vibration_log_fd.write("{}\n".format(','.join(vibration_data)))

    def mav_handle_log_entry(self, m, master, dmd):
        '''handling incoming log entry'''
        if m.time_utc == 0:
            tstring = ''
        else:
            tstring = ctime(m.time_utc)
        self.entries[m.id] = m
        self.download_last_log_num = m.last_log_num
        print("Log %u  numLogs %u lastLog %u size %u %s" % (m.id, m.num_logs, m.last_log_num, m.size, tstring))

    def mav_handle_log_data(self, m, master, dmd):
        '''handling incoming log data'''
        if self.download_file is None:
            return
        # lose some data
        # import random
        # if random.uniform(0,1) < 0.05:
        #    print('dropping ', str(m))
        #    return
        if m.ofs != self.download_ofs:
            self.download_file.seek(m.ofs)
            self.download_ofs = m.ofs
        if m.count != 0:
            s = bytearray(m.data[:m.count])
            self.download_file.write(s)
            self.download_set.add(m.ofs // 90)
            self.download_ofs += m.count
        self.download_last_timestamp = time()
        if m.count == 0 or (m.count < 90 and len(self.download_set) == 1 + (m.ofs // 90)):
            dt = time() - self.download_start
            self.download_file.close()
            size = os.path.getsize(self.download_filename)
            speed = size / (1000.0 * dt)
            print("Finished downloading %s (%u bytes %u seconds, %.1f kbyte/sec %u retries)" % (
                self.download_filename,
                size,
                dt, speed,
                self.retries))
            self.download_file = None
            self.download_filename = None
            self.download_set = set()
            self.download_last_timestamp = None
            self.master.mav.log_request_end_send(self.target_system,
                                                 self.target_component)

    def handle_log_data_missing(self):
        '''handling missing incoming log data'''
        if len(self.download_set) == 0:
            print("handle_log_data_missing(): Nothing downloaded yet?  Returning")
            return
        highest = max(self.download_set)
        diff = set(range(highest)).difference(self.download_set)
        if len(diff) == 0:
            self.master.mav.log_request_data_send(self.target_system,
                                                       self.target_component,
                                                       self.download_lognum, (1 + highest) * 90, 0xffffffff)
            self.retries += 1
        else:
            num_requests = 0
            while num_requests < 20:
                start = min(diff)
                diff.remove(start)
                end = start
                while end + 1 in diff:
                    end += 1
                    diff.remove(end)
                self.master.mav.log_request_data_send(self.target_system,
                                                           self.target_component,
                                                           self.download_lognum, start * 90, (end + 1 - start) * 90)
                num_requests += 1
                self.retries += 1
                if len(diff) == 0:
                    break

    def mav_drop_throttle(self):
        self.rc_outputs[3] = 1000 #Drop the throttle to enable ARMING
        return

    def mav_arm(self):
        #Send command to arm
        self.master.mav.command_long_send(self.target_system, self.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
        return

    def mav_disarm(self):
        #Send command to disarm 
        self.master.mav.command_long_send(self.target_system, self.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
        return

    def mav_download_px4_log_list(self):
        self.download_set = set()
        self.master.mav.log_request_list_send(self.target_system,
                                              self.target_component,
                                              0, 0xffff)

    def mav_download_px4_log(self, log_num, filename):
        '''download a log file'''
        print("Downloading log %u as %s" % (log_num, filename))
        self.download_lognum = log_num
        self.download_file = open(filename, "wb")
        self.download_filename = filename
        self.download_set = set()
        self.download_start = time()
        self.download_last_timestamp = time()
        self.download_ofs = 0
        self.master.mav.log_request_data_send(self.target_system,
                                              self.target_component,
                                              log_num, 0, 0xFFFFFFFF)
        self.retries = 0

    def mav_download_px4_log_latest(self, filename):
        '''download latest log file'''
        if len(self.entries.keys()) == 0:
            print("Please use log list first")
            return
        if self.download_last_log_num == -1:
            print("Please use log list first")
            return
        self.mav_download_px4_log(self.download_last_log_num, filename)

    def idle_task(self):
        '''handle missing log data'''
        if self.download_last_timestamp is not None and time() - self.download_last_timestamp > 0.7:
            self.download_last_timestamp = time()
            self.handle_log_data_missing()

    def run_thread(self):
        while not self.runtime_exit:
            self.read(timeout=0.1)
            self.periodic_triggers()
            self.idle_task()



class MotorPWMDriver:
    def __init__(self, pwmchip, pwmchan, pwmperiod):
        self.pwmchip = pwmchip
        self.pwmchan = pwmchan
        self.pwmperiod = pwmperiod
        self.pwmpath = "{}/pwm{}".format(pwmchip,pwmchan)
        if os.path.exists(pwmchip):
            #Just in case the channel is already enabled, try to disable it
            try:
                self._exec("echo {} > {}/unexport".format(pwmchan,pwmchip))
            except Exception:
                pass
            #Activate the pwm channel
            self._exec("echo {} > {}/export".format(pwmchan,pwmchip))
        self._exec("echo normal > {}/polarity".format(self.pwmpath)) #Set to normal polarity
        self.set_period(pwmperiod)
        self.set_pwm(900000) #Idle motors
        self.enable()

    def __del__(self):
        print("Deleting motorpwm")
        self.disable()
        self._exec("echo {} > {}/unexport".format(self.pwmchan,self.pwmchip))
        return

    def _exec(self, command):
        cp = sp.run("sudo su -c '{}'".format(command), shell=True, stdout=sp.PIPE, check=True)
        return(cp.stdout.decode('utf-8'))

    def _enable_set(self, val):
        self._exec("echo {} > {}/enable".format(val, self.pwmpath))

    def disable(self):
        self._enable_set(0)
    
    def enable(self):
        self._enable_set(1)

    def set_period(self, period):
        self._exec("echo {} > {}/period".format(period, self.pwmpath))

    def set_pwm(self, pwm):
        step_size = 50000
        curr_pwm = int(self._exec("cat {}/duty_cycle".format(self.pwmpath)))
        if( curr_pwm < pwm ):
            while( (pwm - curr_pwm) > step_size): #Don't ramp up too quickly
                curr_pwm = curr_pwm + step_size
                self._exec("echo {} > {}/duty_cycle".format(curr_pwm, self.pwmpath))
                sleep(0.1) #Don't need to sleep as long in ramp up
        else:
            while( (curr_pwm - pwm) > step_size): #Don't ramp down too quickly
                curr_pwm = curr_pwm - step_size
                self._exec("echo {} > {}/duty_cycle".format(curr_pwm, self.pwmpath))
                sleep(0.5)
        self._exec("echo {} > {}/duty_cycle".format(pwm, self.pwmpath))


def set_mav_version(mav10, mav20):
    global mavversion
    '''Set the Mavlink version based on commandline options'''
    #sanity check the options
    if mav10 == True and mav20 == True:
        print("Error: Can't have --mav10 and --mav20 both True")
        sys.exit(1)

    #and set the specific mavlink version (False = autodetect)
    if mav10 == True:
        os.environ['MAVLINK09'] = '1'
        mavversion = "1"
    else:
        os.environ['MAVLINK20'] = '1'
        mavversion = "2"

def process_master(m, mavl):
    '''process packets from the MAVLink master'''
    try:
        s = m.recv(16*1024)
    except Exception:
        sleep(0.1)
        return
    # prevent a dead serial port from causing the CPU to spin. The user hitting enter will
    # cause it to try and reconnect
    if len(s) == 0:
        sleep(0.1)
        return

    msgs = m.mav.parse_buffer(s)
    if msgs:
        for msg in msgs:
            if getattr(m, '_timestamp', None) is None:
                m.post_message(msg)

def main():
    global exit
    global mavl
    global motorpwm
    exit = False
    (opts, args) = parse_args()
    if len(args) != 0:
          print("ERROR: %s takes no position arguments; got (%s)" % (sys.argv[0],str(args)))
          sys.exit(1)
    set_mav_version(opts.mav10, opts.mav20)
    mavutil.set_dialect(opts.dialect)
    mavl = None
    motorpwm = None

    #version information
    if opts.version:
        #pkg_resources doesn't work in the windows exe build, so read the version file
        try:
            import pkg_resources
            version = pkg_resources.require("motorvibe")[0].version
        except:
            start_script = os.path.join(os.environ['LOCALAPPDATA'], "motorvibe", "version.txt")
            f = open(start_script, 'r')
            version = f.readline()

        print("Motorvibe Version: " + version)
        sys.exit(1)

    def quit_handler(signum = None, frame = None):
        global exit
        print('Signal handler called with signal', signum)
        if exit:
            print('Clean shutdown impossible, forcing an exit')
            sys.exit(0)
        else:
            if mavl:
                del(mavl)
            if motorpwm:
                del(motorpwm)
            exit = True

    # Listen for kill signals to cleanly shutdown modules
    fatalsignals = [signal.SIGTERM, signal.SIGINT]
    try:
        fatalsignals.append(signal.SIGHUP)
        fatalsignals.append(signal.SIGQUIT)
    except Exception:
        pass
    for sig in fatalsignals:
        signal.signal(sig, quit_handler)

    if opts.download_logs:
        print("WARNING: Only download px4 logs for each cycle if using a high-speed datalink and not running long cycles.")

    mavl = MotorMavLink(opts.mav10, opts.mav20, opts.SOURCE_SYSTEM, opts.SOURCE_COMPONENT, opts.TARGET_SYSTEM, opts.TARGET_COMPONENT, opts.rtscts, opts.baudrate, opts.master, opts.log)
    motorpwm = MotorPWMDriver(opts.pwmchip, opts.pwmchan, opts.pwmperiod)

    mavl.mav_drop_throttle()
    sleep(5.0)
    mavl.mav_drop_throttle()
    sleep(5.0)
    mavl.mav_drop_throttle()
    sleep(5.0)

    motorpwm.set_pwm(900000)
    
    input_fd = open(opts.input, 'r')
    #Main Loop
    for pwm in input_fd.readlines():
        print("Looping")
        if not os.path.exists(opts.motor):
            os.mkdir(opts.motor)
        pwm = int(pwm.rstrip('\n'))
        pwm_path = "{}/pwm_{}".format(opts.motor,pwm)
        if not os.path.exists(pwm_path):
            os.mkdir(pwm_path)
        motorpwm.set_pwm(pwm)
        mavl.mav_arm() #Unfortunately, we need to arm before we spinup, otherwise preflight checks will fail.
        sleep(5.0) #Give it some time to ramp up and stabilize
        start_time = time()
        mavl.start_vibration_log("{}/{}".format(pwm_path,"vibe.mavlink.csv"))
        while ((time() - start_time)) < opts.runtime:
            sleep(0.5)
        mavl.stop_vibration_log()
        mavl.mav_disarm()
        motorpwm.set_pwm(900000)
        sleep(5.0)
        #Store latest log number to a file for later extraction of sd card (since download of mavlink can take a LOOOONNNNGGG time.
        mavl.px4_log_reset()
        mavl.mav_download_px4_log_list()
        while mavl.download_last_log_num == -1:
            sleep(2)
        lognum_fd = open("{}/lognum.txt".format(pwm_path),'w')
        lognum_fd.write(str(mavl.download_last_log_num))
        lognum_fd.close()
        #Download corresponding px4 log
        if opts.download_logs:
            mavl.mav_download_px4_log_latest("{}/{}".format(pwm_path,"log.ulg"))
            while mavl.download_filename is not None:
                sleep(5) #Logs can take a long time to download: Give a good sleep to let the mavl runtime thread do it's business
    input_fd.close()
    #Cleanup
    exit = True
    del(mavl)
    del(motorpwm)
    sys.exit(0)

if __name__ == "__main__":
    main()
