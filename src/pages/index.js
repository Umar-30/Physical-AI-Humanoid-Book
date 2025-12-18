import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HeroSection() {
  const {siteConfig} = useDocusaurusContext();

  return (
    <div className={styles.heroSection}>
      {/* Background Elements */}
      <div className={styles.backgroundGradient}></div>
      <div className={styles.gridPattern}></div>

      <div className={styles.heroContainer}>
        {/* Left Side - Book Cover */}
        <div className={styles.bookCoverSection}>
          <div className={styles.bookCoverWrapper}>
            <img
              src="/img/physical-ai-cover.svg"
              alt="Physical AI & Humanoid Robotics Book Cover"
              className={styles.bookCover}
            />
            <div className={styles.bookGlow}></div>
          </div>
        </div>

        {/* Right Side - Content */}
        <div className={styles.contentSection}>
          {/* Main Title */}
          <h1 className={styles.mainTitle}>
            <span className={styles.titleLine1}>Physical AI &</span>
            <span className={styles.titleLine2}>Humanoid Robotics</span>
          </h1>

          {/* Welcome Quote */}
          <p className={styles.welcomeQuote}>
            "Welcome to the frontier of intelligent machines — where physical AI meets human-like robotics."
          </p>

          {/* Subtitle */}
          <p className={styles.subtitle}>
            A Hands-On Guide to Building AI-Powered Physical Systems and Humanoid Robotics —
            The Spec & Co-Learning Way
          </p>

          {/* Feature Badges */}
          <div className={styles.featureBadges}>
            <div className={styles.badge}>
              <span className={styles.badgeIcon}>🔓</span>
              <span className={styles.badgeText}>Open Source</span>
            </div>
            <div className={styles.badge}>
              <span className={styles.badgeIcon}>🤖</span>
              <span className={styles.badgeText}>Co-Learning with AI</span>
            </div>
            <div className={styles.badge}>
              <span className={styles.badgeIcon}>📋</span>
              <span className={styles.badgeText}>Spec-Driven Development</span>
            </div>
          </div>

          {/* CTA Buttons */}
          <div className={styles.ctaButtons}>
            <Link
              to="/modules"
              className={clsx(styles.ctaButton, styles.primaryButton)}
            >
              <span className={styles.buttonIcon}>🚀</span>
              <span>Explore Modules</span>
            </Link>
            <Link
              to="/docs/module-1-ros2"
              className={clsx(styles.ctaButton, styles.secondaryButton)}
            >
              <span className={styles.buttonIcon}>📖</span>
              <span>Start Reading</span>
            </Link>
          </div>

          {/* Tech Stack Indicator */}
          <div className={styles.techStack}>
            <div className={styles.techStackLabel}>Built with:</div>
            <div className={styles.techStackItems}>
              <span className={styles.techItem}>ROS 2</span>
              <span className={styles.techDivider}>•</span>
              <span className={styles.techItem}>Gazebo</span>
              <span className={styles.techDivider}>•</span>
              <span className={styles.techItem}>Unity</span>
              <span className={styles.techDivider}>•</span>
              <span className={styles.techItem}>NVIDIA Isaac</span>
              <span className={styles.techDivider}>•</span>
              <span className={styles.techItem}>AI Agents</span>
            </div>
          </div>
        </div>
      </div>

      {/* Scroll Indicator */}
      <div className={styles.scrollIndicator}>
        <div className={styles.scrollMouse}>
          <div className={styles.scrollWheel}></div>
        </div>
        <p className={styles.scrollText}>Scroll to explore</p>
      </div>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Physical AI & Humanoid Robotics - A comprehensive guide to building intelligent robots">
      <HeroSection />
    </Layout>
  );
}
