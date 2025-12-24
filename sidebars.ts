import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

/**
 * Sidebar configuration for Physical AI & Humanoid Robotics book
 */
const sidebars: SidebarsConfig = {
  bookSidebar: [
    'index',
    'quarter-overview',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module1/introduction',
        'module1/ros2-foundations',
        'module1/nodes-topics-services',
        'module1/python-control',
        'module1/ai-ros-integration',
        'module1/urdf-humanoids'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: [
        'module2/introduction',
        'module2/physics-simulation',
        'module2/gazebo-environments',
        'module2/unity-visualization',
        'module2/sensor-simulation',
        'module2/hri-design'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module3/introduction',
        'module3/isaac-sim',
        'module3/isaac-ros-pipelines',
        'module3/vslam',
        'module3/nav2-humanoid',
        'module3/sim-to-real'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module4/introduction',
        'module4/voice-action',
        'module4/whisper-speech',
        'module4/llm-planning',
        'module4/vision-manipulation',
        'module4/cognitive-robotics'
      ],
    },
    'capstone-project',
  ],
};

export default sidebars;
