
import sys, argparse
import setup_mavlink, setup_param
import IPython                      # Debugging purposes

from pymavlink.rotmat import Matrix3, Vector3
from math import sin, cos

from pymavlink.dialects.v10 import ardupilotmega as mavlink

import threading, time


# Main method when called directly
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", help="Serial port or device used for MAVLink communication", default=None)
    args = parser.parse_args()

    m = MAVLinkConn(port=args.port, 
                    mavlink_sys_id=setup_mavlink.MAVLINK_SYSTEM_ID,
                    mavlink_comp_id=setup_mavlink.MAVLINK_COMPONENT_ID,
                    baud=230400)

    if not m.setup():
        error_exit()    # Exit if the MAVLink connection was not set up properly
    
    # m.start_display_thread()
    IPython.embed()
    m.close()
    
def error_exit():
    print("[gopro_test] Something went wrong, exiting...")
    sys.exit(1)

class MAVLinkConn(object):
    """handles communication to gimbal via MAVLink"""
    def __init__(self, port, mavlink_sys_id=1, mavlink_comp_id=154, baud=230400):
        self.port = port
        self.mavserial = None
        self.display_mavlink = True
        self.display_thread = None
        self.display_duration_sec = 0               # how long the GoPro MAVLink messages will be monitored
        self.mavlink_sys_id = mavlink_sys_id
        self.mavlink_comp_id = mavlink_comp_id
        self.baud = baud
        self.gopro_cmds = mavlink.enums['GOPRO_COMMAND']

    def setup(self):
        # Open the serial port
        self.port, self.link = setup_mavlink.open_comm(self.port)
        print("[MAVLinkConn] Connecting via port %s" % self.port)

        if not self.is_gimbal_connected():
            return False

        self.mavserial = self.link.file

        return True

    def is_gimbal_connected(self):
        if setup_mavlink.wait_for_any_gimbal_message(self.link) is None:
            print("[MAVLinkConn] Failed to communicate to gimbal")
            return False
        
        print("[MAVLinkConn] Gimbal connected")
        return True

    def send_control(self, rate=None):
        # Use sample report to generate a sample rate
        #if self.report == None:
        report = self.get_report()  

        Tvg = Matrix3()
        Tvg.from_euler312( report.joint_roll, report.joint_el, report.joint_az)
        current_angle = Vector3(*Tvg.to_euler312())

        if rate == None:
            i = 0
            target = Vector3()
            target.y = 0.4 * (sin(i * 12.5) - 1)
            target.x = 0.2 * cos(i * 2.5)
            target.z = 0.1 * cos(i * 0.5)
                           
            rate = 5 * (target - current_angle)
        
        setup_mavlink.send_gimbal_control(self.link, Tvg.transposed() * rate)

    def reset(self):
        return setup_mavlink.reset_gimbal(self.link)
        
    def flush(self):
        self.mavserial.port.flushInput() # clear any messages in the buffer, so we get a current one

    def receive(self, msg_string=None, blocking=True, timeout=2):
        #while(True):        # does this need to be blocking?
        msg_gimbal = self.mavserial.recv_match(type=msg_string, blocking=True, timeout=2)
        return msg_gimbal

    def get_gopro_heartbeat(self):
        return self.receive(msg_string="GOPRO_HEARTBEAT",timeout=2)

    def get_gopro_get_response(self):
        return self.receive(msg_string="GOPRO_GET_RESPONSE",timeout=2)

    def get_gopro_set_response(self):
        return self.receive(msg_string="GOPRO_SET_RESPONSE",timeout=2)

    def get_report(self):
        return self.receive(msg_string="GIMBAL_REPORT",timeout=1)  

    def display_gopro_msgs(self):
        '''show incoming mavlink GoPro messages'''
        duration = self.display_duration_sec
        end = time.time() + duration
        time_done = False
        print("[display_gopro_msgs] Start monitoring MAVLink...")
        while self.display_mavlink and not time_done:
            curr_time = time.time()
            # Loop forever unless duration is given or display flag is not set
            if duration > 0:
                time_done = (curr_time >= end)

            msg = self.receive()
            if not msg:
                return
            if msg.get_type() == "GOPRO_HEARTBEAT":
                print("[display_gopro_msgs] %s: %s" % (msg.get_type(), mavlink.enums['GOPRO_HEARTBEAT_STATUS'][msg.status].name))
            elif msg.get_type() == "GOPRO_GET_RESPONSE":
                print("[display_gopro_msgs] %s: '%s' = %u" % (msg.get_type(), self.gopro_cmds[msg.cmd_id].name, msg.value))
            elif msg.get_type() == "GOPRO_SET_RESPONSE":
                print("[display_gopro_msgs] %s: '%s' = %u" % (msg.get_type(), self.gopro_cmds[msg.cmd_id].name, msg.result))
            elif msg.get_type().startswith("GOPRO"):
                print("[display_gopro_msgs] %s:" % (msg.get_type()), msg)

        print("[display_gopro_msgs] Finished monitoring MAVLink.")

    def start_display_thread(self, duration_sec=None):
        self.display_mavlink = True
        if duration_sec:
            self.display_duration_sec = duration_sec

        self.display_thread = threading.Thread(target=self.display_gopro_msgs)
        self.display_thread.start()

    def stop_display_thread(self):
        if self.display_thread:
            self.display_mavlink = False
            self.display_thread.join()

    def send_cmd(self, cmd_ind=4, value=None):
        self.start_display_thread(duration_sec=5)
        cmd = self.gopro_cmds[cmd_ind]
        print("[send_cmd] Sending cmd: %s" % cmd.name)
        if value is not None:   # TODO: this should be made more robust with a check of cmd.description for Get/Set
            self.link.gopro_set_request_send(self.mavlink_sys_id, self.mavlink_comp_id, cmd_ind, value)
            print("[send_cmd] set_request sent. (value = %d)" % (value))
        else:
            self.link.gopro_get_request_send(self.mavlink_sys_id, self.mavlink_comp_id, cmd_ind)
            print("[send_cmd] get_request sent.")

    def print_gopro_cmds(self):
        for cmd in self.gopro_cmds:
            print("[display_gopro_cmds] [%d]: %s" % (cmd, self.gopro_cmds[cmd].name))

    def close(self):
        self.stop_display_thread()

if __name__ == '__main__':
    main()
    sys.exit(0)
