#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
# Quitamos el codigo en ROS para dejar solo las fuentes API

import angles
from contextlib import contextmanager
import usb.core
import usb.util
import pyaudio
import math
import numpy as np
import wave
import io
import os
import struct
import sys
import time
from respeaker import Microphone

# suppress error messages from ALSA
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


# Partly copied from https://github.com/respeaker/usb_4_mic_array
# parameter list
# name: (id, offset, type, max, min , r/w, info)
PARAMETERS = {
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
    # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
}


class RespeakerInterface(object):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 100000

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VENDOR_ID,
                                 idProduct=self.PRODUCT_ID)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        print("Initializing Respeaker device")
        self.dev.reset()
        print("Respeaker device initialized (Version: %s)" % self.version)

    def __del__(self):
        try:
            self.close()
        except:
            pass
        finally:
            self.dev = None

    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        response = struct.unpack(b'ii', response.tostring())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')
    
    def set_echo_canceler(self, d):
        self.write('AECFREEZEONOFF', d)

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)

class Respeaker():
    def __init__(self):
        self.update_rate =  10.0
        self.sensor_frame_id =  "respeaker_base"
        self.doa_xy_offset =  0.0
        self.doa_yaw_offset =  90.0
        self.speech_prefetch =  0.5
        self.speech_continuation =  0.5
        self.speech_max_duration =  7.0
        self.speech_min_duration =  0.1
        self.main_channel =  0
        suppress_pyaudio_error =  True
        #
        self.pyaudio = pyaudio.PyAudio()
        self.available_channels = 1
        self.channels = 1 # - 6
        self.device_index =4
        self.rate = 16000
        self.chunk = 1024
        self.bitwidth = 2
        self.bitdepth = 16

        print('Using channels %s' % self.channels)
        self.stream = self.pyaudio.open(
            rate=self.rate,
            format=self.pyaudio.get_format_from_width(self.bitwidth), #pyaudio.paInt16,
            channels= self.channels,
#            input=True, start=False,
            input=True,
            input_device_index= 4, #self.device_index
            frames_per_buffer=self.chunk)
#            stream_callback=self.stream_callback
#        )

        self.respeaker = RespeakerInterface()
        self.speech_audio_buffer = str()
        self.is_speaking = False
        self.recording = False
        self.new_utterance = False
        self.silence_limit = 0.8
        self.silence_counter = 0
        self.prev_is_voice = None
        self.prev_doa = None

        # init config
        self.config = None

        # start
        self.speech_prefetch_bytes = int(
            self.speech_prefetch * self.rate * self.bitdepth / 8.0)
        self.speech_prefetch_buffer = str()
        self.start()

    def on_shutdown(self):
        try:
            self.respeaker.close()
        except:
            pass
        finally:
            self.respeaker = None
        try:
            self.respeaker_audio.stop()
        except:
            pass
        finally:
            self.respeaker_audio = None

    def on_config(self, config):
        if self.config is None:
            for name in config.keys():
                config[name] = self.respeaker.read(name)
        else:
            for name, value in config.items():
                prev_val = self.config[name]
                if prev_val != value:
                    self.respeaker.write(name, value)
        self.config = config
        return config

    def on_audio(self, data):
            self.is_speaking = self.respeaker.is_voice()
            if self.is_speaking:
                self.on_timer()
                if not self.recording:
                    print("🎤 Detectada voz, grabando utterance...")
                    self.recording = True
                    self.silence_counter = 0
                    if len(self.speech_audio_buffer) == 0:
                        self.speech_audio_buffer = self.speech_prefetch_buffer
                    self.speech_audio_buffer += data
            else:
                self.speech_prefetch_buffer += data
                self.speech_prefetch_buffer = self.speech_prefetch_buffer[-self.speech_prefetch_bytes:]
                if self.recording:
                    self.silence_counter += self.respeaker_audio.chunk / self.respeaker_audio.rate
                    if self.silence_counter > self.silence_limit:
                        print("🛑 Fin de utterance. Avisando a Reconocedor...")
                        self.recording = False
                        self.silence_counter = 0
                        self.on_utterance(self)
                        self.new_utterance = True

    def on_utterance(self):
        # ya hay frase, llamamos al reconocedor
        frames = self.speech_audio_buffer
        if not frames:
            return None
        if len(b''.join(frames)) % 2 != 0:
            frames[-1] = frames[-1][:-1]
    #    audio_bytes = b''.join(frames)
    #    audio_stream = io.BytesIO(audio_bytes)
        buf = io.BytesIO()
        wf = wave.open(buf, 'wb')
        wf.setnchannels(self.respeaker_audio.channels)
        wf.setsampwidth(self.respeaker_audio.bitwidth)  # 16 bits
        wf.setframerate(self.respeaker_audio.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        buf.seek(0)    
        self.audio_stream = buf

    def on_timer(self, event):
#        stamp = event.current_real or rospy.Time.now()
        is_voice = self.respeaker.is_voice()
        doa_rad = math.radians(self.respeaker.direction - 180.0)
        doa_rad = angles.shortest_angular_distance(
            doa_rad, math.radians(self.doa_yaw_offset))
        doa = math.degrees(doa_rad)

        # vad
        if is_voice != self.prev_is_voice:
            print("VAD is voice:{}", is_voice)
            self.prev_is_voice = is_voice

        # doa
        if doa != self.prev_doa:
            print("DOA:{%d}", doa)
            self.prev_doa = doa

            print(self.sensor_frame_id)
            print(self.doa_xy_offset * np.cos(doa_rad))
            print(self.doa_xy_offset * np.sin(doa_rad))

    def rec_audio(self):
        data = self.stream.read(self.chunk, exception_on_overflow=False)
        self.on_audio(data)

    def play_audio(self, data):
        self.stream.write(data)
        self.stream.stop_stream()
        self.stream.close()

    def __del__(self):
        try:
            self.stop()
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

#    def stream_callback(self, in_data, frame_count, time_info, status):
#        for i in range(0, int(self.rate / self.chunk * )):
#            data = stream.read(CHUNK)
#            frames.append(data)
#

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()

def main(args=None):
  rs = Respeaker()
#  rs.on_config(None)
  rs.respeaker.set_echo_canceler(0)
  while True:
      rs.rec_audio()

if __name__ == '__main__':
    main()

