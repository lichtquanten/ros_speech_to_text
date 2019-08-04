#!/usr/bin/env python
from ibm_watson import SpeechToTextV1
import io
import numpy as np
import rospy
import soundfile as sf
import yaml

from audio_io_msgs.msg import AudioData
from speech_to_text_msgs.msg import Transcript
from std_msgs.msg import Header

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

def main():
    rospy.init_node('s2t_sync', anonymous=True)

    # Get parameters
    input_topic = rospy.get_param('~input_topic')
    credentials_path = rospy.get_param('~credentials_path')
    format = rospy.get_param('~format', default='PCM')

    # Get credentials
    with open(credentials_path) as cf:
        credentials = yaml.safe_load(cf)
    speech_to_text = SpeechToTextV1(
        iam_apikey=credentials['apikey'],
        url=credentials['url']
    )

    def callback(msg, pub):
        if format == 'FLAC':
            content_type = 'audio/flac'
            dtype = width_to_dtype(msg.sample_width)
            data = np.fromstring(msg.data, dtype)
            with io.BytesIO() as flac_file:
                sf.write(
                    flac_file,
                    data,
                    msg.sample_rate,
                    format='FLAC'
                )
                data = str(flac_file.getvalue())
        else:
            endianness = 'big-endian' if msg.is_bigendian else 'little-endian'
            content_type = """audio/l16; rate={}; channels={}; endianness={}""".format(msg.sample_rate,
            msg.num_channels, endianness)
            data = str(msg.data)
        # Get results from Watson
        result = speech_to_text.recognize(
            audio=data,
            content_type=content_type
        ).get_result()
        if not result['results']:
            return
        # Convert to transcript message
        result = result['results'][0]['alternatives'][0]
        transcript = result['transcript']
        confidence = result['confidence']
        out_msg = Transcript(transcript=transcript, confidence=confidence)
        # Use timestamp from the audio messagae
        out_msg.header.stamp = msg.header.stamp
        pub.publish(out_msg)

    pub = rospy.Publisher('~transcript', Transcript, queue_size=10)
    rospy.Subscriber(input_topic, AudioData, callback, callback_args=pub)
    rospy.spin()

if __name__ == '__main__':
     main()
