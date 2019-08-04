#!/usr/bin/env python
from ibm_watson import SpeechToTextV1
from ibm_watson.websocket import AudioSource, RecognizeCallback
import io
import numpy as np
from Queue import Queue
import rospy
import soundfile as sf
import sys
import threading
import yaml

from audio_io_msgs.msg import AudioData
from speech_to_text_msgs.msg import Transcript

def width_to_dtype(width):
	if width == 1:
		return np.int8
	if width == 2:
		return np.int16
	if width == 4:
		return np.int32
	if width == 8:
		return np.int64
	return None

class MyRecognizeCallback(RecognizeCallback):
    def __init__(self, final_topic, interim_topic):
        RecognizeCallback.__init__(self)
        self._pub_final = rospy.Publisher(final_topic, Transcript, queue_size=10)
        self._pub_interim = rospy.Publisher(interim_topic, Transcript, queue_size=10)

    def on_connected(self):
        rospy.loginfo('Connected to Watson.')

    def on_listening(self):
        rospy.loginfo('Watson is listening.')

    def on_transcription(self, result):
        if not result:
            return
        transcript = result[0]['transcript']
        confidence = result[0]['confidence']
        msg = Transcript(transcript=transcript, confidence=confidence)
        msg.header.stamp = rospy.Time.now()
        self._pub_final.publish(msg)

    def on_hypothesis(self, hypothesis):
        msg = Transcript(transcript=hypothesis, confidence=0)
        msg.header.stamp = rospy.Time.now()
        self._pub_interim.publish(msg)

    def on_error(self, error):
        rospy.logerr('WATSON: ' + error)

    def on_inactivity_error(self, error):
        rospy.logerr('WATSON: ' + error)

    def on_close(self):
        rospy.loginfo('Watson is closed.')

def main():
    rospy.init_node('s2t_rt', anonymous=True)
    # Get parameters
    input_topic = rospy.get_param('~input_topic')
    credentials_path = rospy.get_param('~credentials_path')
    format = rospy.get_param('~format', 'PCM')

    # Get credentials
    with open(credentials_path) as cf:
        credentials = yaml.safe_load(cf)

    speech_to_text = SpeechToTextV1(
        iam_apikey=credentials['apikey'],
        url=credentials['url']
    )
    queue = Queue(maxsize=10)
    audio_source = AudioSource(queue, is_recording=True, is_buffer=True)
    recognize_callback = MyRecognizeCallback('~transcript', '~interim')

    msg = rospy.wait_for_message(input_topic, AudioData)
    if format == 'FLAC':
        content_type = 'audio/flac'
    else:
        # Get content type from message
        endianness = 'big-endian' if msg.is_bigendian else 'little-endian'
        content_type = """audio/l16; rate={}; channels={};
        endianness={}""".format(msg.sample_rate,
        msg.num_channels, endianness)
    recognizer = speech_to_text.recognize_using_websocket(audio=audio_source,
                                             content_type=content_type,
                                             recognize_callback=recognize_callback,
                                             interim_results=True,
                                             inactivity_timeout=-1)
    recognize_thread = threading.Thread(target=recognizer.start, args=())
    recognize_thread.daemon = True
    recognize_thread.start()

    def callback(msg):
        if format == 'FLAC':
    			dtype = width_to_dtype(msg.sample_width)
    			data = np.fromstring(msg.data, dtype)
    			with io.BytesIO() as flac_file:
    				sf.write(
                        flac_file,
                        data,
                        msg.sample_rate,
                        format=format
                    )
    				queue.put(str(ogg_file.getvalue()))
        else:
            queue.put(str(msg.data))
    rospy.Subscriber(input_topic, AudioData, callback)
    rospy.spin()
    recognizer.close()

if __name__ == '__main__':
     main()
