import React from 'react';
import Link from '@docusaurus/Link'; // Assuming Docusaurus Link component for navigation
import styles from './CourseSection.module.css'; // Import CSS Modules for styling

const CourseSection = () => {
  const modules = [
    {
      number: 1,
      title: "The Robotic Nervous System (ROS 2)",
      icon: "⚙️", // Gear icon for middleware/system
      summary: "Explore ROS 2, the core middleware for robot control. Learn to connect Python agents with robot controllers and master URDF for humanoid robot modeling.",
      link: "/docs/category/module-1-ros2", // Placeholder link
    },
    {
      number: 2,
      title: "The Digital Twin (Gazebo & Unity)",
      icon: "🌐", // Globe/network icon for digital twin/simulation
      summary: "Build immersive simulated environments with Gazebo and Unity. Understand advanced physics, realistic rendering, and multi-sensor integration for robust robot development.",
      link: "docs/category/module-2-digital-twin", // Placeholder link
    },
    {
      number: 3,
      title: "The AI-Robot Brain (NVIDIA Isaac™)",
      icon: "🧠", // Brain icon for AI/intelligence
      summary: "Harness NVIDIA Isaac Sim for photorealistic simulation and AI training. Implement advanced perception with Isaac ROS and develop bipedal navigation using Nav2.",
      link: "/docs/category/module-3-ai-brain", // Placeholder link
    },
    {
      number: 4,
      title: "Vision-Language-Action (VLA)",
      icon: "🤖", // Robot icon for VLA/humanoid
      summary: "Integrate cutting-edge LLMs with robotics to enable intuitive voice commands. Develop cognitive planning capabilities, culminating in a fully autonomous humanoid project.",
      link: "/docs/category/module-4-vla", // Placeholder link
    },
  ];

  return (
    <section className={styles.courseSectionContainer}>
      {/* Background gradient effect */}
      <div className={styles.backgroundEffect}></div>
      
      <div className={styles.mainContentWrapper}>
        {/* Welcome Note Section */}
        <div className={styles.welcomeNoteSection}>
          <h2 className={styles.welcomeTitle}>
            Welcome to the Frontier of Intelligent Machines!
          </h2>
          <p className={styles.welcomeText}>
            Welcome, future roboticists, to the cutting edge of intelligent machines! This course is your gateway to mastering Physical AI and Humanoid Robotics, equipping you with the skills to build and control advanced autonomous systems. Prepare to dive deep into ROS 2, Digital Twins, NVIDIA Isaac™, and Vision-Language-Action pipelines, transforming complex concepts into real-world innovations.
          </p>
          <Link
            className={styles.startJourneyButton}
            to="docs/category/module-1-ros2" // Link to main course introduction or getting started
          >
            Start Your Robotics Journey →
          </Link>
        </div>

        {/* Module Cards Section */}
        <h3 className={styles.moduleSectionTitle}>
          Course Modules
        </h3>
        
        <div className={styles.moduleGrid}>
          {modules.map((module) => (
            <div
              key={module.number}
              className={styles.moduleCard}
            >
              {/* Neon border effect on hover */}
              <div className={styles.moduleCardBorderEffect}></div>
              
              <div className={styles.moduleCardContent}>
                <div className={styles.moduleCardHeader}>
                  <span className={styles.moduleNumberBadge}>
                    Module {module.number}
                  </span>
                  <span className={styles.moduleIcon}>
                    {module.icon}
                  </span>
                </div>
                <h4 className={styles.moduleCardTitle}>
                  {module.title}
                </h4>
                <p className={styles.moduleCardSummary}>
                  {module.summary}
                </p>
                <div className={styles.moduleCardButtonWrapper}>
                  <Link
                    className={styles.moduleCardButton}
                    to={module.link}
                  >
                    View Details
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default CourseSection;