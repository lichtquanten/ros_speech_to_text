ROS Watson Speech
=================

## Overview

This is a ROS wrapper for IBM Watson's Speech to Text API.

## Nodes

### 1. Synchronous

A `synchronous` node will send the data in each received audio message to Watson's API. The node's callbacks are run concurrently, so simultaneous requests will not block. The timestamp of the published transcript will match that of the input `AudioData` message.

Example usage with ouput from [ros_vad](https://github.com/sean-hackett/ros_vad):

```bash
rosrun watson_speech synchronous.py _input_topic:=/vad/complete _credentials_path:=credentials.json
```

#### Published Topics

**~transcript** ([speech_to_text_msgs/Transcript](https://github.com/sean-hackett/ros_speech_to_text/blob/master/speech_to_text_msgs/msg/Transcript.msg)): Transcript result from Watson.

#### Parameters

**~input_topic** ([audio_io_msgs/AudioData](https://github.com/sean-hackett/audio_io_msgs/blob/master/msg/AudioData.msg)): Input audio topic. Must be 16-bit. May be mono or stereo, may be little or big-endian.

**~credentials_path** (string): Absolute or relative path to a JSON file containing the fields `apikey` and `url`. See [here](https://cloud.ibm.com/docs/services/watson?topic=watson-iam) for info on authenticating with Watson.

**~format** (string, default: 'PCM'): Format to convert audio to before uploading to Watson. Only 'PCM' and 'FLAC' are supported as of now. Data will not be changed if using PCM.

If parsing your credentials fails, check that your fields are two spaces in from the left.

### 2. Real-time

Warning: This is an experimental feature. As of now, it works for ~10 minutes and then segfaults.

Data from received `AudioData` messages is streamed to Watson's API over a websocket. The timestamp of published transcripts is determined by the time of receiving Watson's response.

Data must be received at a near real-time rate else the Watson API will timeout. If less than 15 seconds of audio are sent in any 30 second period of time, a `sesssion timeout` will occur. See [here](https://cloud.ibm.com/docs/services/speech-to-text?topic=speech-to-text-input#timeouts-session) for more information on session timeouts.

Example usage with ouput from [audio_io](https://github.com/sean-hackett/audio_io/blob/master/audio_io):

```bash
rosrun watson_speech realtime.py _input_topic:=/mic/data _credentials_path:=credentials.json
```

#### Published Topics

**~transcript** ([speech_to_text_msgs/Transcript](https://github.com/sean-hackett/ros_speech_to_text/blob/master/speech_to_text_msgs/msg/Transcript.msg)): Transcript result from Watson.

**~interim** ([speech_to_text_msgs/Transcript](https://github.com/sean-hackett/ros_speech_to_text/blob/master/speech_to_text_msgs/msg/Transcript.msg)): Interim transcript results from Watson. Confidence not provided and thus set to 0 for interim results. Note that this topic does not specify when a result is final.

#### Parameters

**~input_topic** ([audio_io_msgs/AudioData](https://github.com/sean-hackett/audio_io_msgs/blob/master/msg/AudioData.msg)): Input audio topic. Must be 16-bit. May be mono or stereo, may be little or big-endian.

**~credentials_path** (string): Absolute or relative path to a JSON file containing the fields `apikey` and `url`. See [here](https://cloud.ibm.com/docs/services/watson?topic=watson-iam) for info on authenticating with Watson.

**~format** (string, default: 'PCM'): Format to convert audio to before uploading to Watson. Only 'PCM' and 'FLAC' are supported as of now. Data will not be changd if using PCM.

If parsing your credentials fails, check that your fields are two spaces in from the left.

## Dependencies

If you wish to use real-time speech to text, use my fork of the IBM Watson Python SDK, available [here](https://github.com/sean-hackett/python-sdk).
