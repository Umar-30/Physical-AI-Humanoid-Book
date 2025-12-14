import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'generated-index',
        title: 'Module 1: The Robotic Nervous System (ROS 2)',
        description: 'Learn about the fundamentals of ROS 2 architecture and core concepts.',
        slug: '/category/module-1-ros2',
      },
      items: [
        'module-1-ros2/chapter-1-1-introduction-to-ros2-architecture',
        'module-1-ros2/chapter-1-2-ros2-core-concepts-nodes-topics-services',
        'module-1-ros2/chapter-1-3-python-robotics-with-rclpy',
        'module-1-ros2/chapter-1-4-humanoid-modeling-with-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'generated-index',
        title: 'Module 2: The Digital Twin (Gazebo & Unity)',
        description: 'Explore simulation environments for robotics, including Gazebo and Unity.',
        slug: '/category/module-2-digital-twin',
      },
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/environment-setup',
        'module-2-digital-twin/gazebo-installation',
        'module-2-digital-twin/unity-installation',
        'module-2-digital-twin/chapter-2-1-gazebo-physics-simulation-fundamentals',
        'module-2-digital-twin/chapter-2-2-advanced-urdf-for-simulation',
        'module-2-digital-twin/chapter-2-3-unity-for-human-robot-interaction',
        'module-2-digital-twin/chapter-2-4-multi-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      link: {
        type: 'generated-index',
        title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
        description: 'Dive into AI-powered robotics with NVIDIA Isaac Sim and Isaac ROS.',
        slug: '/category/module-3-ai-brain',
      },
      items: [
        'module-3-ai-brain/index',
        'module-3-ai-brain/environment-setup',
        // Placeholder for Chapter 1 content
        // Placeholder for Chapter 2 content
        // Placeholder for Chapter 3 content
        // Placeholder for Chapter 4 content
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action (VLA)',
        description: 'Understand how to integrate vision, language, and action for advanced robotic capabilities.',
        slug: '/category/module-4-vla',
      },
      items: [
        'module-4-vla/chapter-4.1-voice-interface-with-openai-whisper',
        'module-4-vla/chapter-4.2-llm-based-cognitive-planning',
        'module-4-vla/chapter-4.3-vla-pipeline-integration',
        'module-4-vla/chapter-4.4-capstone-project-autonomous-humanoid',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      link: {
        type: 'generated-index',
        title: 'Appendices',
        description: 'Supplementary materials and reference guides.',
        slug: '/category/appendices',
      },
      items: [
        'appendix/appendix-a-hardware-requirements-setup-guide',
        'appendix/appendix-b-environment-setup-troubleshooting',
        'appendix/appendix-c-ros2-command-reference',
        'appendix/appendix-d-project-repository-templates',
      ],
    },
  ],
};

export default sidebars;