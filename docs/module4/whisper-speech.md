# Whisper for Speech Recognition

## Introduction to Whisper in Robotics

OpenAI's Whisper represents a breakthrough in automatic speech recognition (ASR) technology, offering robust, multilingual speech recognition capabilities that are particularly valuable for robotic applications. Whisper's ability to handle diverse audio conditions, multiple languages, and noisy environments makes it an ideal choice for voice-enabled robotic systems operating in real-world environments.

In the context of Physical AI and humanoid robotics, Whisper provides the foundation for natural human-robot interaction through speech. Its advanced neural architecture and training on diverse datasets enable it to maintain accuracy even in challenging acoustic conditions typical of robotic environments, including motor noise, environmental sounds, and reverberation.

## Whisper Architecture and Capabilities

### Transformer-Based Architecture
Whisper utilizes a transformer-based architecture for speech recognition:

- **Encoder**: Processes audio input through multiple attention layers
- **Decoder**: Generates text output conditioned on audio representations
- **Multilingual Training**: Trained on 98 languages for broad applicability
- **Robustness**: Designed to handle various audio conditions and accents

### Model Variants
Different Whisper model variants offer trade-offs between accuracy and performance:

- **tiny**: Smallest model, fastest inference, lower accuracy
- **base**: Balanced model for general use
- **small**: Good accuracy with reasonable performance
- **medium**: High accuracy with moderate computational requirements
- **large**: Highest accuracy, most computationally intensive

## Whisper Integration with ROS 2

### ROS 2 Node Implementation
Whisper can be integrated into ROS 2 as a dedicated speech recognition node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
from std_msgs.msg import String
import whisper
import numpy as np
import torch
from io import BytesIO
import wave

class WhisperROSNode(Node):
    def __init__(self):
        super().__init__('whisper_ros_node')

        # Audio input subscription
        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10
        )

        # Speech recognition result publisher
        self.transcription_pub = self.create_publisher(String, 'transcription', 10)

        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_size', 'base'),
                ('language', 'en'),
                ('device', 'cuda' if torch.cuda.is_available() else 'cpu'),
                ('real_time_mode', True),
                ('energy_threshold', 4000)
            ]
        )

        # Load Whisper model
        model_size = self.get_parameter('model_size').value
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size, device=self.get_parameter('device').value)

        # Audio buffer for real-time processing
        self.audio_buffer = []
        self.buffer_size = 16000 * 2  # 2 seconds of audio at 16kHz

    def audio_callback(self, msg):
        # Process incoming audio data
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            if self.get_parameter('real_time_mode').value:
                # Real-time processing: add to buffer and process when sufficient
                self.audio_buffer.extend(audio_array)

                if len(self.audio_buffer) >= self.buffer_size:
                    # Process accumulated audio
                    self.process_audio_segment(self.audio_buffer[:self.buffer_size])
                    # Keep overlapping segment for continuity
                    self.audio_buffer = self.audio_buffer[self.buffer_size//2:]
            else:
                # Process immediately
                self.process_audio_segment(audio_array)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def process_audio_segment(self, audio_segment):
        # Convert audio to appropriate format for Whisper
        audio_tensor = torch.from_numpy(audio_segment).to(self.model.device)

        # Transcribe audio using Whisper
        result = self.model.transcribe(
            audio_tensor,
            language=self.get_parameter('language').value,
            fp16=self.model.device.type == 'cuda'
        )

        if result['text'].strip():
            # Publish transcription
            transcription_msg = String()
            transcription_msg.data = result['text'].strip()
            self.transcription_pub.publish(transcription_msg)
            self.get_logger().info(f'Transcribed: {result["text"]}')
```

### Audio Preprocessing Pipeline
Whisper integration requires proper audio preprocessing:

- **Format Conversion**: Converting ROS audio messages to Whisper-compatible format
- **Sample Rate Normalization**: Ensuring consistent 16kHz sample rate
- **Noise Reduction**: Preprocessing to improve recognition quality
- **Voice Activity Detection**: Identifying speech segments for processing

## Advanced Whisper Features for Robotics

### Multilingual Support
Whisper's multilingual capabilities enable global robotics applications:

```python
class MultilingualWhisperNode(WhisperROSNode):
    def __init__(self):
        super().__init__()
        self.language_detection_enabled = True
        self.supported_languages = ['en', 'es', 'fr', 'de', 'ja', 'ko', 'zh']

    def process_audio_segment(self, audio_segment):
        audio_tensor = torch.frombuffer(audio_segment, dtype=np.float32).to(self.model.device)

        if self.language_detection_enabled:
            # Detect language automatically
            segment = self.model.encode(audio_tensor)
            results = self.model.detect_language(segment)
            detected_language = results[0][0].split('-')[0]  # Extract language code

            if detected_language in self.supported_languages:
                result = self.model.transcribe(audio_tensor, language=detected_language)
            else:
                result = self.model.transcribe(audio_tensor, language='en')
        else:
            result = self.model.transcribe(audio_tensor, language=self.get_parameter('language').value)

        # Publish with language metadata
        transcription_msg = String()
        transcription_msg.data = f"[{detected_language if self.language_detection_enabled else self.get_parameter('language').value}] {result['text']}"
        self.transcription_pub.publish(transcription_msg)
```

### Speaker Diarization
Identifying different speakers in multi-person interactions:

- **Speaker Segmentation**: Identifying speaker changes in audio
- **Speaker Embeddings**: Creating unique identifiers for speakers
- **Multi-speaker Processing**: Handling conversations with multiple people
- **Focus Selection**: Determining which speaker to respond to

## Performance Optimization

### Real-Time Processing
Optimizing Whisper for real-time robotic applications:

- **Chunked Processing**: Processing audio in small chunks for low latency
- **Model Quantization**: Reducing model size for faster inference
- **GPU Acceleration**: Leveraging GPU computation when available
- **Caching**: Caching intermediate results for faster subsequent processing

### Resource Management
Managing computational resources effectively:

```python
class OptimizedWhisperNode(WhisperROSNode):
    def __init__(self):
        super().__init__()

        # Adaptive processing based on system load
        self.system_monitor = SystemMonitor()
        self.min_confidence = 0.7
        self.adaptive_threshold = True

    def process_audio_segment(self, audio_segment):
        # Check system load and adjust processing accordingly
        system_load = self.system_monitor.get_cpu_load()

        if system_load > 0.8:
            # High load: use faster, less accurate processing
            result = self.model.transcribe(
                audio_segment,
                language=self.get_parameter('language').value,
                fp16=self.model.device.type == 'cuda',
                temperature=0.7  # More deterministic
            )
        else:
            # Normal load: use full accuracy
            result = self.model.transcribe(
                audio_segment,
                language=self.get_parameter('language').value,
                fp16=self.model.device.type == 'cuda',
                temperature=[0.0, 0.2, 0.4, 0.6, 0.8, 1.0]  # Multiple attempts
            )

        # Filter low-confidence results
        if result.get('confidence', 1.0) >= self.min_confidence:
            transcription_msg = String()
            transcription_msg.data = result['text']
            self.transcription_pub.publish(transcription_msg)
```

## Integration with Physical AI Systems

### Voice Command Processing
Integrating Whisper output with command processing systems:

- **Intent Recognition**: Converting transcriptions to actionable commands
- **Entity Extraction**: Identifying objects, locations, and parameters
- **Context Integration**: Using environmental context to improve understanding
- **Error Recovery**: Handling recognition errors gracefully

### Multi-Modal Fusion
Combining speech recognition with other modalities:

```python
class MultiModalWhisperNode(WhisperROSNode):
    def __init__(self):
        super().__init__()

        # Subscribe to other modalities
        self.vision_sub = self.create_subscription(
            String, 'vision_objects', self.vision_callback, 10
        )
        self.location_sub = self.create_subscription(
            String, 'current_location', self.location_callback, 10
        )

        self.current_objects = []
        self.current_location = "unknown"

    def vision_callback(self, msg):
        self.current_objects = msg.data.split(',')

    def location_callback(self, msg):
        self.current_location = msg.data

    def process_audio_segment(self, audio_segment):
        # Transcribe audio
        result = self.model.transcribe(audio_segment, language=self.get_parameter('language').value)

        # Enhance with contextual information
        enhanced_command = self.enhance_with_context(result['text'])

        # Publish enhanced transcription
        transcription_msg = String()
        transcription_msg.data = enhanced_command
        self.transcription_pub.publish(transcription_msg)

    def enhance_with_context(self, text):
        # Use context to disambiguate commands
        enhanced_text = text
        if 'object' in text.lower():
            if self.current_objects:
                enhanced_text += f" (visible objects: {', '.join(self.current_objects)})"
        if 'here' in text.lower():
            enhanced_text = text.replace('here', f'at {self.current_location}')

        return enhanced_text
```

## Validation and Testing

### Speech Recognition Accuracy
Assessing Whisper performance in robotic environments:

- **Word Error Rate (WER)**: Percentage of incorrectly recognized words
- **Real-time Performance**: Latency and throughput measurements
- **Robustness Testing**: Performance under various noise conditions
- **Language Coverage**: Accuracy across supported languages

### Robot-Specific Validation
Validation for humanoid robotics applications:

- **Environmental Testing**: Performance in typical robot operating environments
- **Integration Testing**: End-to-end voice command processing
- **Safety Validation**: Ensuring safe handling of incorrect recognitions
- **User Experience**: Subjective assessment of recognition quality