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
              src="https://imageio.forbes.com/specials-images/imageserve/66bee357cf48b97789cbc270/0x0.jpg?format=jpg&height=900&width=1600&fit=bounds"
              alt="Physical AI & Humanoid Robotics Book Cover"
              className={styles.bookCover}
            />
            <div className={styles.bookGlow}></div>
          </div>
        </div>

        {/* Right Side - Content */}
        <div className={styles.contentSection}>
          {/* Main Title - Large Bold Heading */}
          <h1 className={styles.mainTitle}>
            Physical AI &<br />
            <span className={styles.titleHighlight}>Humanoid Robotics</span>
          </h1>

          {/* Bold Subtitle */}
          <p className={styles.subtitle}>
            A Hands-On Guide to Building AI-Powered Physical Systems and Humanoid Robotics —
            The Spec & Co-Learning Way
          </p>

          {/* Welcome Quote */}
          <p className={styles.welcomeQuote}>
            "Welcome to the frontier of intelligent machines — where physical AI meets human-like robotics."
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
        </div>
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
