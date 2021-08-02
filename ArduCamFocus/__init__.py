# coding=utf-8
from __future__ import absolute_import

import octoprint.plugin
import smbus

import re

import cv2

import requests
try:
    from StringIO import StringIO
except ImportError:
    from io import StringIO

try:
    from urllib.parse import urlparse
except ImportError:
    from urlparse import urlparse

from contextlib import closing
try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen

import numpy as np

class MjpegStreamChunker:

    def __init__(self):
        self.boundary = None
        self.current_chunk = StringIO()

    # Return: mjpeg chunk if found
    #         None: in the middle of the chunk
    def findMjpegChunk(self, line):
        if not self.boundary:   # The first time endOfChunk should be called with 'boundary' text as input
            self.boundary = line
            self.current_chunk.write(line)
            return None

        if len(line) == len(self.boundary) and line == self.boundary:  # start of next chunk
            return self.current_chunk.getvalue()

        self.current_chunk.write(line)
        return None

class ArduCamFocusPlugin(octoprint.plugin.SettingsPlugin,
                         octoprint.plugin.AssetPlugin,
                         octoprint.plugin.TemplatePlugin,
                         octoprint.plugin.StartupPlugin):

	def __init__(self):
		self.bus = None

	def on_after_startup(self):
		try:
			self.bus = smbus.SMBus(0)
		except:
			self._plugin_manager.send_plugin_message(self._identifier, dict(error="Unable to open SMBUS"))

		self.current_focus = self._settings.get_int(["FOCUS"])

	##~~ SettingsPlugin mixin
	def get_settings_defaults(self):
		return dict(
			FOCUS="100"
		)

	def on_settings_save(self, data):
		oldFOCUS = self._settings.get_int(["FOCUS"])

		octoprint.plugin.SettingsPlugin.on_settings_save(self, data)

		newFOCUS = self._settings.get_int(["FOCUS"])

		if oldFOCUS != newFOCUS:
			self._logger.info("FOCUS changed, initilizing to %d." % (int(newFOCUS)))
			self.current_focus = newFOCUS


	##~~ AssetPlugin mixin

	def get_assets(self):
		# Define your plugin's asset files to automatically include in the
		# core UI here.
		return dict(
			js=["js/ArduCamFocus.js"],
		)

	##-- Template hooks

	def get_template_configs(self):
		return [dict(type="generic",custom_bindings=False)]

	##~~ Softwareupdate hook

	def get_update_information(self):
		return dict(
			ArduCamFocus=dict(
				displayName="ArduCamFocus",
				displayVersion=self._plugin_version,

				# version check: github repository
				type="github_release",
				user="moof-src",
				repo="ArduCamFocus",
				current=self._plugin_version,

				# update method: pip
				pip="https://github.com/moof-src/ArduCamFocus/archive/{target_version}.zip"
			)
		)

	##~~ Utility functions
	def focus (self, f):
		if f < 100:
			f = 100
		elif f > 1000:
			f = 1000
		value = (f << 4) & 0x3ff0
		data1 = (value >> 8) & 0x3f
		data2 = value & 0xf0
		if self.bus:
			self._logger.info("setting FOCUS to %d" % (f))
			write_attempts = 10
			while write_attempts:
				try:
					self.bus.write_byte_data(0xc, data1, data2)
				except IOError:
					write_attempts -= 1
				else:
					break
			if not write_attempts:
				self._plugin_manager.send_plugin_message(self._identifier, dict(error="Trouble accessing camera. I2C bus failure.  Is camera plugged in?"))
			self.current_focus = f
			self._settings.set_int(["FOCUS"], f, min=100, max=1000)
			self._settings.save()
			self._plugin_manager.send_plugin_message(self._identifier, dict(focus_val=self.current_focus))
		else:
			self._plugin_manager.send_plugin_message(self._identifier, dict(error="unable to use SMBus/I2C"))

	def webcam_full_url(self, url):
		if not url or not url.strip():
			return None

		full_url = url.strip()
		if not urlparse(full_url).scheme:
			full_url = "http://localhost/" + re.sub(r"^\/", "", full_url)

		return full_url

	def capture_jpeg(self, webcam_settings):
		snapshot_url = self.webcam_full_url(webcam_settings.get("snapshot", ''))

		if snapshot_url:
			snapshot_validate_ssl = bool(webcam_settings.get("snapshotSslValidation", 'False'))

			r = requests.get(snapshot_url, stream=True, timeout=5, verify=snapshot_validate_ssl )
			r.raise_for_status()
			jpg = r.content
			return jpg

		else:
			stream_url = self.webcam_full_url(webcam_settings.get("stream", "/webcam/?action=stream"))

			with closing(urlopen(stream_url)) as res:
				chunker = MjpegStreamChunker()

				while True:
					data = res.readline()
					mjpg = chunker.findMjpegChunk(data)
					if mjpg:
						res.close()
						mjpeg_headers_index = mjpg.find('\r\n'*2)
						if mjpeg_headers_index > 0:
							return mjpg[mjpeg_headers_index+4:]
						else:
							raise Exception('Wrong mjpeg data format')


	def laplacian(self, img):
		#img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
		img_sobel = cv2.Laplacian(img,cv2.CV_16U)
		return cv2.mean(img_sobel)[0]

	def calculation(self):
		#rawCapture = PiRGBArray(self.camera)
		#self.camera.capture(rawCapture,format="bgr", use_video_port=True)
		#image = rawCapture.array
		#rawCapture.truncate(0)

		jpeg_image = self.capture_jpeg(self._settings.global_get(["webcam"]))
		image = np.asarray(bytearray(jpeg_image), dtype="uint8")
		return self.laplacian(cv2.imdecode(image, cv2.IMREAD_GRAYSCALE))

	def autofocus(self):
		focal_distance = self._settings.get_int(["FOCUS"])
		if focal_distance < 110:
			focal_distance = 110
		if focal_distance > 990:
			focal_distance = 990;

		self.focus(focal_distance)
		cur_clarity = self.calculation()

		self.focus(focal_distance - 10);
		negative_clarity = self.calculation();
		self.focus(focal_distance + 10);
		positive_clarity = self.calculation();

		if cur_clarity > negative_clarity and cur_clarity > positive_clarity:
			return; # we already have the best focus

		#which way is better?
		rate = 10
		if(negative_clarity > positive_clarity):
			rate = rate * -1

		max_index = focal_distance
		max_clarity = cur_clarity
		last_clarity = cur_clarity
		dec_count = 0

		while True:
			focal_distance = focal_distance + rate
			self.focus(focal_distance)
			cur_clarity = self.calculation()
			if cur_clarity > max_clarity:
				max_index = focal_distance
				max_clarity = cur_clarity

			#If the image clarity starts to decrease
			if cur_clarity < last_clarity:
				dec_count += 1
			else:
				dec_count = 0
			#Image clarity is reduced by six consecutive frames
			if dec_count > 6:
				break
			last_clarity = cur_clarity

			#Increase the focal distance
			if focal_distance > 1000 or focal_distance < 100:
				break

		final_focus = max_index + (dec_count * rate)
		self.focus(final_focus)

	##~~ atcommand hook

	def processAtCommand(self, comm_instance, phase, command, parameters, tags=None, *args, **kwargs):
		if command == 'ARDUCAMFOCUS':
			try:
				self.focus(self.current_focus + int(parameters))
			except ValueError:
				self._logger.info("unknown parameter %s" % parameters)
				return
		elif command == 'ARDUCAMFOCUSSET':
			try:
				self.focus(int(parameters))
			except ValueError:
				self._logger.info("unknown parameter %s" % parameters)
				return
		elif command == 'ARDUCAMAUTOFOCUS':
			self.autofocus()

__plugin_name__ = "ArduCamFocus"
__plugin_pythoncompat__ = ">=2.7,<4"

def __plugin_load__():
	global __plugin_implementation__
	__plugin_implementation__ = ArduCamFocusPlugin()

	global __plugin_hooks__
	__plugin_hooks__ = {
		"octoprint.plugin.softwareupdate.check_config": __plugin_implementation__.get_update_information,
		"octoprint.comm.protocol.atcommand.queuing": __plugin_implementation__.processAtCommand
	}
