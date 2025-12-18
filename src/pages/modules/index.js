import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

const modules = [
  {
    number: '01',
    title: 'The Robotic Nervous System (ROS 2)',
    focus: 'Middleware for robot control.',
    bullets: [
      'ROS 2 Nodes, Topics, and Services.',
      'Bridging Python Agents to ROS controllers using rclpy.',
      'Understanding URDF (Unified Robot Description Format) for humanoids.',
    ],
    link: '/docs/module-1-ros2',
  },
  {
    number: '02',
    title: 'The Digital Twin (Gazebo & Unity)',
    focus: 'Physics simulation and environment building.',
    bullets: [
      'Simulating physics, gravity, and collisions in Gazebo.',
      'High-fidelity rendering and human-robot interaction in Unity.',
      'Simulating sensors: LiDAR, Depth Cameras, and IMUs.',
    ],
    link: '/docs/module-2/',
  },
  {
    number: '03',
    title: 'The AI-Robot Brain (NVIDIA Isaac™)',
    focus: 'Advanced perception and training.',
    bullets: [
      'NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.',
      'Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.',
      'Nav2: Path planning for bipedal humanoid movement.',
    ],
    link: '#',
  },
  {
    number: '04',
    title: 'Vision-Language-Action (VLA)',
    focus: 'The convergence of LLMs and Robotics.',
    bullets: [
      'Voice-to-Action: Using OpenAI Whisper for voice commands.',
      'Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.',
      'Capstone Project: The Autonomous Humanoid — voice command → plan → navigate → identify → manipulate.',
    ],
    link: '#',
  },
];

function WelcomeSection() {
  return (
    <header className={styles.welcomeSection}>
      <div className={styles.welcomeContainer}>
        <h1 className={styles.welcomeTitle}>
          Welcome to the Physical AI & Humanoid Robotics Course
        </h1>
        <p className={styles.welcomeSubtitle}>
          In this course, you will explore foundational robotics concepts, AI-driven behaviors,
          simulation, perception, and the Spec-driven approach to building intelligent physical systems.
          Follow the modules below to progress from middleware and simulation to advanced perception
          and vision-language action.
        </p>
        <div className={styles.welcomeDivider}></div>
      </div>
    </header>
  );
}

function ModuleCard({ module }) {
  return (
    <article className={styles.moduleCard} aria-label={`Module ${module.number}: ${module.title}`}>
      <div className={styles.moduleHeader}>
        <span className={styles.moduleNumber}>{module.number}</span>
      </div>
      <h2 className={styles.moduleTitle}>{module.title}</h2>
      <p className={styles.moduleFocus}>
        <strong>Focus:</strong> {module.focus}
      </p>
      <ul className={styles.moduleBullets}>
        {module.bullets.map((bullet, idx) => (
          <li key={idx}>{bullet}</li>
        ))}
      </ul>
      <Link
        to={module.link}
        className={styles.moduleButton}
        aria-label={`Start ${module.title}`}
      >
        Start Module →
      </Link>
    </article>
  );
}

function ModulesGrid() {
  return (
    <main className={styles.modulesSection}>
      <div className={styles.modulesContainer}>
        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <ModuleCard key={idx} module={module} />
          ))}
        </div>
      </div>
    </main>
  );
}

export default function ModulesPage() {
  return (
    <Layout
      title="Course Modules"
      description="Explore the four core modules of Physical AI & Humanoid Robotics">
      <div className={styles.modulesPage}>
        <WelcomeSection />
        <ModulesGrid />
      </div>
    </Layout>
  );
}
