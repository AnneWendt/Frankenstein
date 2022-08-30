#!/usr/bin/env python
"""
This file contains the tools to read out the video stream from the camera.
Usage: create a new instance of the Video class with the correct UDP port.
Then you can read out frames and show them with OpenCV.
"""

import gi  # GStreamer
import numpy as np

gi.require_version('Gst', '1.0')  # GStreamer needs to be version 1.0 or higher
from gi.repository import Gst


class Video():
    """
    Video capture class.
    Decodes the video stream coming through Gstreamer via specified UDP port
    and saves it to 'frame' variable.
    """

    def __init__(self, udp_port=5610):
        """
        Start up GStreamer connection.
        Args:
            udp_port (int, optional): the UDP port for the video stream.
            (can be set on the companion in home/pi/gstreamer2-extra.params)
        """
        Gst.init(None)

        self.port = udp_port
        # this will contain the current video frame
        self.__frame = None
        # says if we have a new/unused frame available
        self.new_sample = False

        # set some video specs
        # video from RasPi needs to be converted from YUV to BGR format
        # it will come through the pipe and be stored in the sink
        # http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format
        udp_src = 'udpsrc port={}'.format(self.port)
        video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        sink_config = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        # now start GST pipe with these settings
        specs = ' '.join([udp_src, video_codec, video_decode, sink_config])
        self.video_pipe = Gst.parse_launch(specs)
        self.video_pipe.set_state(Gst.State.PLAYING)

        # connect the pipe to the sink and stream into the callback function
        self.video_sink = self.video_pipe.get_by_name('appsink0')
        # https://gstreamer.freedesktop.org/documentation/applib/gstappsink.html?gi-language=c#GstAppSink::new-sample
        self.video_sink.connect('new-sample', self.sink_callback)

    def sink_callback(self, sink):
        """
        This method is called automatically by GStreamer when a new frame is
        available. Pulls video from the sink, converts it to a useable format
        and saves it in our private __frame variable.
        """
        sample = sink.emit('pull-sample')
        self.new_sample = True

        # comes as byte array, needs to be converted to numpy array
        buf = sample.get_buffer()
        caps = sample.get_caps()
        self.__frame = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)

        # important for the GStreamer library
        return Gst.FlowReturn.OK

    def frame_available(self):
        """
        Says if we have a frame ready. The video stream takes a little while
        to start up, so this function can be used to check if it's done yet.
        """
        return self.__frame is not None and self.new_sample

    @property
    def frame(self):
        """
        Public access to the current frame.
        """
        self.new_sample = False
        return self.__frame
