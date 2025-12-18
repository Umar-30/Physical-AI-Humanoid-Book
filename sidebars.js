/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Main sidebar for Module 1
  moduleSidebar: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-1-ros2/index',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Architecture',
          link: {
            type: 'doc',
            id: 'module-1-ros2/architecture/index',
          },
          items: [
            'module-1-ros2/architecture/middleware-role',
            'module-1-ros2/architecture/ros2-overview',
            'module-1-ros2/architecture/ros1-vs-ros2',
            'module-1-ros2/architecture/dds-explained',
            'module-1-ros2/architecture/getting-started',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Communication Primitives',
          link: {
            type: 'doc',
            id: 'module-1-ros2/communication/index',
          },
          items: [
            'module-1-ros2/communication/topics-pubsub',
            'module-1-ros2/communication/services-reqrep',
            'module-1-ros2/communication/actions-goals',
            'module-1-ros2/communication/qos-profiles',
            'module-1-ros2/communication/executors',
            'module-1-ros2/communication/graph-design',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Python Implementation',
          link: {
            type: 'doc',
            id: 'module-1-ros2/python-rclpy/index',
          },
          items: [
            'module-1-ros2/python-rclpy/rclpy-overview',
            'module-1-ros2/python-rclpy/package-setup',
            'module-1-ros2/python-rclpy/publishers-subscribers',
            'module-1-ros2/python-rclpy/services',
            'module-1-ros2/python-rclpy/actions',
            'module-1-ros2/python-rclpy/custom-messages',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: URDF for Humanoid Robots',
          link: {
            type: 'doc',
            id: 'module-1-ros2/urdf/index',
          },
          items: [
            'module-1-ros2/urdf/urdf-basics',
            'module-1-ros2/urdf/links-joints',
            'module-1-ros2/urdf/coordinate-frames',
            'module-1-ros2/urdf/humanoid-urdf',
            'module-1-ros2/urdf/visualization',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'doc',
        id: 'module-2-digital-twin/module-2-overview',
      },
      items: [
        {
          type: 'category',
          label: 'Chapter 5: Gazebo Basics',
          link: {
            type: 'doc',
            id: 'module-2-digital-twin/chapter-5-gazebo-basics/chapter-5-overview',
          },
          items: [
            'module-2-digital-twin/chapter-5-gazebo-basics/gazebo-installation',
            'module-2-digital-twin/chapter-5-gazebo-basics/sdf-world-files',
            'module-2-digital-twin/chapter-5-gazebo-basics/robot-spawning',
            'module-2-digital-twin/chapter-5-gazebo-basics/physics-configuration',
            'module-2-digital-twin/chapter-5-gazebo-basics/troubleshooting',
            'module-2-digital-twin/chapter-5-gazebo-basics/chapter-5-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: URDF Physics',
          link: {
            type: 'doc',
            id: 'module-2-digital-twin/chapter-6-urdf-physics/chapter-6-overview',
          },
          items: [
            'module-2-digital-twin/chapter-6-urdf-physics/sensor-integration',
            'module-2-digital-twin/chapter-6-urdf-physics/collision-geometry',
            'module-2-digital-twin/chapter-6-urdf-physics/inertia-calculation',
            'module-2-digital-twin/chapter-6-urdf-physics/joint-configuration',
            'module-2-digital-twin/chapter-6-urdf-physics/validation',
            'module-2-digital-twin/chapter-6-urdf-physics/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 7: Unity Rendering',
          link: {
            type: 'doc',
            id: 'module-2-digital-twin/chapter-7-unity-rendering/chapter-7-overview',
          },
          items: [
            'module-2-digital-twin/chapter-7-unity-rendering/unity-setup',
            'module-2-digital-twin/chapter-7-unity-rendering/ros-integration',
            'module-2-digital-twin/chapter-7-unity-rendering/scene-creation',
            'module-2-digital-twin/chapter-7-unity-rendering/human-avatars',
            'module-2-digital-twin/chapter-7-unity-rendering/ui-overlays',
            'module-2-digital-twin/chapter-7-unity-rendering/exercises',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 8: Multi-Simulator',
          link: {
            type: 'doc',
            id: 'module-2-digital-twin/chapter-8-multi-simulator/chapter-8-overview',
          },
          items: [
            'module-2-digital-twin/chapter-8-multi-simulator/architecture',
            'module-2-digital-twin/chapter-8-multi-simulator/launch-coordination',
            'module-2-digital-twin/chapter-8-multi-simulator/synchronization',
            'module-2-digital-twin/chapter-8-multi-simulator/performance',
            'module-2-digital-twin/chapter-8-multi-simulator/troubleshooting',
            'module-2-digital-twin/chapter-8-multi-simulator/exercises',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;
