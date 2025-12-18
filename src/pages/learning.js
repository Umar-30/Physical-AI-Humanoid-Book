import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Layout from '@theme/Layout';
import styles from './learning.module.css';

function WelcomeSection() {
  return (
    <section className={styles.welcomeSection}>
      <div className="container">
        <div className={styles.welcomeBox}>
          <h1 className={styles.welcomeTitle}>
            <span className={styles.neonText}>Welcome to the</span>
            <br />
            <span className={styles.glowText}>Learning Portal</span>
          </h1>
          <p className={styles.welcomeDescription}>
            Embark on a journey to master humanoid robotics through three comprehensive modules.
            Each module builds upon the previous, taking you from fundamentals to advanced AI integration.
          </p>
          <div className={styles.progressInfo}>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>3</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>20+</span>
              <span className={styles.statLabel}>Chapters</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>100+</span>
              <span className={styles.statLabel}>Examples</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      number: '01',
      title: 'The Robotic Nervous System',
      subtitle: 'ROS 2 Fundamentals',
      description: 'Master the middleware powering modern robotics. Learn nodes, topics, services, actions, and build your first robot agents with Python.',
      technologies: ['ROS 2', 'Python', 'rclpy', 'URDF'],
      icon: '🧠',
      color: '#00f0ff', // Cyan
      link: '/docs/module-1-ros2',
      status: 'active',
      progress: 100
    },
    {
      number: '02',
      title: 'The Digital Twin',
      subtitle: 'Gazebo & Unity Simulation',
      description: 'Build production-ready digital twins with Gazebo physics and Unity rendering. Create realistic simulations synchronized via ROS 2.',
      technologies: ['Gazebo', 'Unity', 'Physics', 'Sensors'],
      icon: '🎮',
      color: '#ff00ff', // Magenta
      link: '/docs/module-2-digital-twin/module-2-overview',
      status: 'active',
      progress: 80
    },
    {
      number: '03',
      title: 'The AI-Robot Brain',
      subtitle: 'NVIDIA Isaac™',
      description: 'Advanced perception and training with NVIDIA Isaac Sim, hardware-accelerated VSLAM, and Nav2 path planning for bipedal movement.',
      technologies: ['Isaac Sim', 'Isaac ROS', 'VSLAM', 'Nav2'],
      icon: '🤖',
      color: '#00ff00', // Neon Green
      link: '#',
      status: 'coming-soon',
      progress: 0
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      subtitle: 'VLA Integration',
      description: 'The convergence of LLMs and Robotics. Voice commands, cognitive planning, and autonomous humanoid capstone project.',
      technologies: ['Whisper', 'LLMs', 'Voice-to-Action', 'Planning'],
      icon: '🗣️',
      color: '#ffff00', // Yellow
      link: '#',
      status: 'coming-soon',
      progress: 0
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className={styles.sectionTitle}>
          <span className={styles.titleLine}></span>
          Choose Your Module
          <span className={styles.titleLine}></span>
        </h2>

        <div className={styles.modulesGrid}>
          {modules.map((module, idx) => (
            <Link
              key={idx}
              to={module.link}
              className={clsx(
                styles.moduleCard,
                module.status === 'coming-soon' && styles.moduleCardDisabled
              )}
              style={{
                '--module-color': module.color,
                pointerEvents: module.status === 'coming-soon' ? 'none' : 'auto'
              }}
            >
              {/* Card Header */}
              <div className={styles.moduleHeader}>
                <span className={styles.moduleNumber}>{module.number}</span>
                {module.status === 'coming-soon' && (
                  <span className={styles.comingSoonBadge}>Coming Soon</span>
                )}
              </div>

              {/* Card Icon */}
              <div className={styles.moduleIcon}>{module.icon}</div>

              {/* Card Content */}
              <div className={styles.moduleContent}>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
                <p className={styles.moduleSubtitle}>{module.subtitle}</p>
                <p className={styles.moduleDescription}>{module.description}</p>

                {/* Technologies */}
                <div className={styles.moduleTechnologies}>
                  {module.technologies.map((tech, techIdx) => (
                    <span key={techIdx} className={styles.techTag}>
                      {tech}
                    </span>
                  ))}
                </div>

                {/* Progress Bar */}
                {module.status === 'active' && (
                  <div className={styles.progressBar}>
                    <div
                      className={styles.progressFill}
                      style={{width: `${module.progress}%`}}
                    ></div>
                  </div>
                )}
              </div>

              {/* Hover Effect */}
              <div className={styles.cardGlow}></div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  return (
    <section className={styles.pathSection}>
      <div className="container">
        <h2 className={styles.pathTitle}>Your Learning Journey</h2>
        <div className={styles.pathTimeline}>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>1</div>
            <div className={styles.stepContent}>
              <h4>Foundation</h4>
              <p>Learn ROS 2 basics and robot architecture</p>
            </div>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>2</div>
            <div className={styles.stepContent}>
              <h4>Simulation</h4>
              <p>Build digital twins with Gazebo & Unity</p>
            </div>
          </div>
          <div className={styles.pathArrow}>→</div>
          <div className={styles.pathStep}>
            <div className={styles.stepNumber}>3</div>
            <div className={styles.stepContent}>
              <h4>Intelligence</h4>
              <p>Integrate AI agents and machine learning</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Learning() {
  return (
    <Layout
      title="Learning Portal"
      description="Choose your learning module and start building intelligent humanoid robots">
      <div className={styles.learningPage}>
        <WelcomeSection />
        <ModulesSection />
        <LearningPath />
      </div>
    </Layout>
  );
}
